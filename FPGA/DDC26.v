// ============================================================================
// DDC26.v  -- uDSR receive-only DDC, 10-bit ADC, 81 MHz sample clock
//
// Derived from DDC23.v. Fixes carried over from the uDST's DDC25.v, the timing
// repair for the Fmax regression those fixes caused, and (new in DDC26) a
// gain re-plan so the 16-bit output no longer costs sensitivity.
// Each change is tagged // *** FIX n ***
//
//   FIX 1  CIC integrators widened 40 -> 48 bits to MATCH the comb chain.
//          A CIC only works because the integrators' modulo-2^W wraparound is
//          exactly undone by the combs, and that requires integrators and
//          combs to be the SAME width. With 40-bit integrators feeding 48-bit
//          combs, every integrator wrap is sign-extended into the wider comb
//          arithmetic instead of cancelling. A plain 16-bit output slice hid
//          this for a long time; the saturation of FIX 3 exposed it.
//
//   FIX 2  Round-half-up before the mixer's right shift (>>>13 since FIX 5,
//          originally >>>15). A truncating shift puts a systematic -0.5 LSB
//          DC pedestal on every mixer sample, which the CIC then multiplies
//          by its full passband gain.
//
//   FIX 3  Saturate-then-slice on the CIC output instead of the bare
//          cicI_reg[15:0] truncation, with the rounding constant derived from
//          a parameterised shift.
//
//   FIX 4  TIMING. FIX 3 as originally written put a 48-bit adder, a 33-bit
//          reduction AND/NOR and an output mux between y2I/y2Q and
//          cicI_reg/cicQ_reg in ONE clock -- 10 logic levels, 13.28 ns, which
//          dropped Fmax to 73.1 MHz (all 30 violating endpoints were in this
//          single path group). Three changes, none of which alter the
//          arithmetic by a single LSB:
//            (a) the round-and-shift is now a narrow increment, not a 48-bit
//                add: (y2 + 2^(S-1)) >>> S  ==  y2[47:S] + y2[S-1], exactly.
//                (S = 24 since FIX 5: a 24-bit increment.)
//            (b) the "fits in 16 bits" test looks only at the bits left above
//                bit 15 after the shift (9 bits at S = 24) instead of 33 --
//                the bits shifted away were sign copies of each other and
//                carried no information.
//            (c) the round/shift and the saturate/slice sit in SEPARATE
//                pipeline stages. This costs one 81 MHz clock on a path that
//                produces a sample once every 432 clocks.
//          The same (a)+(b) narrowing is applied to the mixer scaler and the
//          FIR output stage for margin; the FIR's pipeline depth
//          (ROUND_LATENCY, and hence CNT_LAST) is unchanged.
//
//   FIX 5  GAIN RE-PLAN (DDC26). With the DDC25 gains (mixer >>>15, CIC
//          >>>23, overall x16) the receiver noise at the output was only
//          ~0.35 LSB rms per channel -- smaller than the rounding noise added
//          by the mixer, CIC and FIR output stages -- so quantisation roughly
//          doubled the noise power (~3.1 dB sensitivity loss), while a
//          full-scale ADC sine reached only half of the 16-bit range (6 dB of
//          headroom that bought nothing, since the ADC clips first anyway).
//            (a) MIX_SHIFT 15 -> 13. The 10-bit sample sits in the 12-bit slot
//                with two zero LSBs, so keeping two more fraction bits at the
//                mixer is free; mixer rounding noise drops 12 dB (x4 gain).
//            (b) CIC_SHIFT 23 -> 24 (x9.611 -> x4.806).
//          Net: overall gain x16 -> x32, so a full-scale ADC sine now gives
//          |IQ| = 32768, i.e. the FIR output saturation coincides with ADC
//          clipping and no headroom is wasted. Quantisation penalty falls from
//          ~3.1 dB to ~1.0 dB (the remainder is the 16-bit CIC and FIR output
//          rounding, removable only with a wider output word). The CIC output
//          for a full-scale tone is still +/-19684, so the CIC saturation
//          still never fires in normal operation. x32 matches the scaling the
//          uDST runs at CIC >>>22.
//
//          *** PICO SIDE: IQ counts are now 6 dB (x2) larger than DDC23 for
//          the same input. Re-check S-meter, AGC and display-gain calibration.
//
// Unchanged: the NCO, the FIR coefficients, the decimation ratios (432 x 6),
// and all pipeline latencies.
// ============================================================================

module nco_sincos_rx #(
    parameter LUT_BITS = 10,          // 1024 entries
    parameter PHASE_BITS = 32
)(
    input  wire                   clk,
    input  wire                   rst,
    input  wire [PHASE_BITS-1:0]  tuning_word,
    output reg  signed [15:0]     sin_out,
    output reg  signed [15:0]     cos_out
);

    localparam LUT_SIZE = 1 << LUT_BITS;
    // Sized to LUT_BITS so sin_idx + COS_OFFSET is a LUT_BITS-wide add that
    // wraps mod LUT_SIZE by construction (an unsized parameter widens the
    // expression and draws a truncation warning, EX3791).
    localparam [LUT_BITS-1:0] COS_OFFSET = LUT_SIZE >> 2;  // +pi/2

    // ----------------------------
    // Phase accumulator
    // ----------------------------
    reg [PHASE_BITS-1:0] phase;

    always @(posedge clk) begin
        if (rst) begin
            phase <= 0;
        end else begin
            phase <= phase + tuning_word;
        end
    end

    // ----------------------------
    // LUT indexing
    // ----------------------------
    // :- is starting bits and width eg, 31 :- 10
    wire [LUT_BITS-1:0] sin_idx = phase[PHASE_BITS-1 -: LUT_BITS];
    wire [LUT_BITS-1:0] cos_idx = sin_idx + COS_OFFSET;   // wraps mod LUT_SIZE

    // ----------------------------
    // Sine LUT (Q15)
    // ----------------------------
    // Two copies of the table so sine and cosine can be read in the same cycle.
    // The reads are REGISTERED: a block RAM read is synchronous, so a registered
    // read maps cleanly to BRAM. An asynchronous (combinational) read of a
    // 1024x16 ROM does NOT infer BRAM and is built from a large, slow mux of
    // logic cells instead.
    reg signed [15:0] sin_lut [0:LUT_SIZE-1];
    reg signed [15:0] cos_lut [0:LUT_SIZE-1];
	initial begin
		$readmemh("sin_lut.hex", sin_lut);
		$readmemh("sin_lut.hex", cos_lut);
	end

    always @(posedge clk) begin
        if (rst) begin
            sin_out <= 0;
            cos_out <= 0;
        end else begin
            sin_out <= sin_lut[sin_idx];
            cos_out <= cos_lut[cos_idx];
        end
    end

endmodule

// Registered 16x16 signed multiplier (32-bit product). Latency = 3 clocks.
//
// The INPUTS are registered (a_r, b_r) so the multiply runs register-to-
// register and packs into the DSP block (input regs + multiplier pipeline
// reg + output reg). That register placement is what lets it close timing at
// ~81 MHz; registering only the output would leave the full combinational
// multiply in one clock.
module mac16_reg (
    input  wire               clk,
    input  wire               rst,
    input  wire signed [15:0] a,        // multiplicand
    input  wire signed [15:0] b,        // multiplier (coefficient)
    output reg  signed [31:0] prod_out
);
    reg signed [15:0] a_r, b_r;         // input registers   -> DSP A/B regs
    reg signed [31:0] mult_r;           // multiplier output -> DSP pipeline reg

    always @(posedge clk) begin
        if (rst) begin
            a_r      <= 16'sd0;
            b_r      <= 16'sd0;
            mult_r   <= 32'sd0;
            prod_out <= 32'sd0;
        end else begin
            // stage 0 -> 1: register inputs
            a_r <= a;
            b_r <= b;

            // stage 1 -> 2: multiply (register-to-register)
            mult_r <= $signed(a_r) * $signed(b_r);

            // stage 2 -> output
            prod_out <= mult_r;
        end
    end
endmodule

// 252-tap, decimate-by-6, RX anti-alias/channel FIR (81 MHz variant).
//
// Coefficients carry ~x1.665 DC gain (Q15 sum = 54552) so the overall RX gain
// is mixer(x4) * CIC(x4.806) * FIR(x1.665) = x32.0 (see FIX 5), the same as
// the 80 MHz uDST at CIC >>>22 (CIC x32 * FIR x1). A full-scale ADC sine gives
// |IQ| = 32768, so this output saturation is where the receiver clips -- at
// ADC full scale, by design. Output is saturated to 16 bits (the >1 FIR gain
// can otherwise overflow acc>>>15).
// Q15 realised stopband ~ -67 dB; passband flat to 14 kHz; droop @14k = 0.24 dB.
// One serial multiplier per channel; the sample histories and the coefficient
// table live in BLOCK RAM (synchronous read), because 252-deep x 16-bit x 2
// histories will not fit in flip-flops on a small device.
//
// Pipeline latency accounting (this is the part that must be exact):
//   - present a tap address on MAC clock k
//   - BRAM registered read  -> operands valid on clock k+READ_LAT
//   - mac16_reg (3 stages)  -> product valid on clock k+READ_LAT+MUL_LATENCY
//   - round/shift register  -> +ROUND_LATENCY
//   We accumulate the product every clock and latch the result once the last
//   real tap has drained, i.e.
//   CNT_LAST = NTAPS + READ_LAT + MUL_LATENCY + ROUND_LATENCY.
//
// The coefficient address is forced to the zero-padded ROM entry whenever we
// are not issuing a valid tap, so every spurious product in the pipeline is 0
// and accumulating on every clock is safe.
module fir252_dec6_rx (
    input  wire               clk,
    input  wire               rst,
    input  wire               din_valid,
    input  wire signed [15:0] xi,
    input  wire signed [15:0] xq,
    output reg                dout_valid,
    output reg signed [15:0]  yoI,
    output reg signed [15:0]  yoQ
);
    localparam integer NTAPS       = 252;
    localparam integer BUF_LEN     = 256;   // power of two; >= NTAPS
    localparam integer READ_LAT    = 1;     // BRAM synchronous-read latency
    localparam integer MUL_LATENCY = 3;     // mac16_reg latency
    localparam integer ROUND_LATENCY = 1;   // the accI_sh register below
    localparam [9:0]   CNT_LAST    = NTAPS + READ_LAT + MUL_LATENCY + ROUND_LATENCY;  // = 257
    localparam [7:0]   ZERO_TAP    = 8'd252; // ROM entry that holds 0 (252..255 pad)

    // ---- coefficient ROM (256 entries; [252..255] = 0 pad), synchronous read ----
    reg signed [15:0] Hmem [0:255];
    initial $readmemh("fir_rx252_coeffs.hex", Hmem);

    // ---- sample histories in BRAM (synchronous read) ----
    reg signed [15:0] dI [0:BUF_LEN-1];
    reg signed [15:0] dQ [0:BUF_LEN-1];
    integer bi;
    initial for (bi = 0; bi < BUF_LEN; bi = bi + 1) begin
        dI[bi] = 0;
        dQ[bi] = 0;   // BRAM powers up to 0 on Gowin/iCE40; matched here for sim
    end

    reg [7:0] wr;        // next write slot
    reg [7:0] base;      // newest-sample index for the current MAC run
    reg [2:0] decim;     // 0..5 decimate-by-6 counter
    reg [9:0] cnt;       // MAC step counter

    localparam S_IDLE = 1'b0, S_MAC = 1'b1;
    reg state;

    // address generation
    wire issuing = (state == S_MAC) && (cnt < NTAPS);
    wire [7:0] coeff_addr = issuing ? cnt[7:0] : ZERO_TAP;  // 0 coeff when idle/draining
    wire [7:0] rd_addr    = base - cnt[7:0];                // don't-care unless issuing
    wire       wr_en      = (state == S_IDLE) && din_valid;

    // registered reads (infer BRAM). Reset the read-output regs so they
    // can't briefly go X right after reset and poison the (self-accumulating)
    // FIR accumulator for an entire decimation period -- same class of fix
    // applied to the TX FIR (DUC) earlier in this project.
    reg signed [15:0] sampI, sampQ, coeff;
    always @(posedge clk) begin
        if (wr_en) begin
            dI[wr] <= xi;
            dQ[wr] <= xq;
        end
        if (rst) begin
            sampI <= 0; sampQ <= 0; coeff <= 0;
        end else begin
            sampI <= dI[rd_addr];
            sampQ <= dQ[rd_addr];
            coeff <= Hmem[coeff_addr];
        end
    end

    // one multiplier per channel; same coefficient feeds both
    wire signed [31:0] prodI, prodQ;
    mac16_reg macI (.clk(clk), .rst(rst), .a(sampI), .b(coeff), .prod_out(prodI));
    mac16_reg macQ (.clk(clk), .rst(rst), .a(sampQ), .b(coeff), .prod_out(prodQ));

    // ---- accumulator: its own block, UNCONDITIONAL add so the FSM state
    //      never enters the carry chain; the clear is a post-add mux (1 LUT). ----
    //
    // Width check: worst case |acc| = 2^15 (input) * 54552 (Q15 coeff sum)
    //            = 1.79e9 -> 31 bits + sign. ACCW = 40 has ample headroom.
    localparam integer ACCW = 40;
    reg signed [ACCW-1:0] accI, accQ;
    wire acc_clr = (state == S_IDLE) && din_valid && (decim == 3'd5);  // start of each dec-6 group
    always @(posedge clk) begin
        if (rst || acc_clr) begin
            accI <= 0; accQ <= 0;
        end else begin
            accI <= accI + prodI;   // clean add; clr selects 0 vs sum AFTER the adder
            accQ <= accQ + prodQ;
        end
    end

    // ------------------------------------------------------------------
    // *** FIX 4 *** output round + shift, narrowed
    // ------------------------------------------------------------------
    // Round-half-up on a right shift by S is (x + 2^(S-1)) >>> S, which is
    // IDENTICAL to  x[MSB:S] + x[S-1].  Proof: write x = 2^S*A + B with
    // A = x>>>S (floor division) and B = x[S-1:0]; then
    //   floor((x + 2^(S-1)) / 2^S) = A + floor((B + 2^(S-1))/2^S)
    //                              = A + (B >= 2^(S-1)) = A + x[S-1].
    // So the 40-bit adder becomes a 25-bit increment, and the register that
    // holds the result shrinks from 40 bits to 25.
    localparam integer FIR_SHIFT = 15;
    localparam integer FIRSHW    = ACCW - FIR_SHIFT;   // 25 bits after the shift

    wire signed [FIRSHW-1:0] accI_hi = $signed(accI[ACCW-1:FIR_SHIFT]);
    wire signed [FIRSHW-1:0] accQ_hi = $signed(accQ[ACCW-1:FIR_SHIFT]);
    wire                     accI_rb = accI[FIR_SHIFT-1];   // round bit
    wire                     accQ_rb = accQ[FIR_SHIFT-1];

    reg signed [FIRSHW-1:0] accI_sh, accQ_sh;   // already rounded AND shifted
    always @(posedge clk) begin
        if (rst) begin
            accI_sh <= 0; accQ_sh <= 0;
        end else begin
            accI_sh <= accI_hi + $signed({{(FIRSHW-1){1'b0}}, accI_rb});
            accQ_sh <= accQ_hi + $signed({{(FIRSHW-1){1'b0}}, accQ_rb});
        end
    end

    // saturate-then-slice. "Fits in signed 16 bits" == every bit above bit 15
    // equals the sign bit -- 10 bits to test after the narrowing above.
    wire yI_fit = (&accI_sh[FIRSHW-1:15]) | (~|accI_sh[FIRSHW-1:15]);
    wire yQ_fit = (&accQ_sh[FIRSHW-1:15]) | (~|accQ_sh[FIRSHW-1:15]);

    wire signed [15:0] yI_sat = yI_fit ? accI_sh[15:0]
                                       : (accI_sh[FIRSHW-1] ? -16'sd32768 : 16'sd32767);
    wire signed [15:0] yQ_sat = yQ_fit ? accQ_sh[15:0]
                                       : (accQ_sh[FIRSHW-1] ? -16'sd32768 : 16'sd32767);

    // ---- control FSM: no longer touches accI/accQ ----
    always @(posedge clk) begin
        if (rst) begin
            wr <= 0; base <= 0; decim <= 0; cnt <= 0;
            yoI <= 0; yoQ <= 0; dout_valid <= 0; state <= S_IDLE;
        end else begin
            dout_valid <= 1'b0;
            case (state)
            S_IDLE: begin
                if (din_valid) begin
                    if (decim == 3'd5) begin
                        decim <= 0; base <= wr; cnt <= 0; state <= S_MAC;
                    end else decim <= decim + 1'b1;
                    wr <= wr + 1'b1;
                end
            end
            S_MAC: begin
                if (cnt == CNT_LAST) begin
                    yoI <= yI_sat; yoQ <= yQ_sat;
                    dout_valid <= 1'b1; state <= S_IDLE;
                end else cnt <= cnt + 1'b1;
            end
            endcase
        end
    end
endmodule

// 10-BIT ADC VARIANT: adc_in is 10 bits (e.g. AD9215). The sample is
// MSB-ALIGNED into the old 12-bit position ({adc,2'b00}) at the mixer input
// register, so the ADC full scale is still +/-2048 counts and the FIR
// coefficient set is unchanged. The quantisation step is 4 counts instead of
// 1, which is what leaves two spare LSBs for the mixer to keep (FIX 5).
//
// NOTE: adc_in is declared signed, so whatever drives this port must already
// be two's complement. The AD9215 defaults to offset binary unless the DFS pin
// is tied high -- if it is left in offset-binary mode, invert the MSB at the
// pad before it reaches here.
module ddc_top (
    input  wire        clk80,
    input  wire        rst,
    input  wire signed [9:0] adc_in,
    // tuning word interface
    input  wire        tw_valid,
    input  wire [31:0] tuning_word,
    output wire        iq_valid,
    output wire signed [15:0] iq_i,
    output wire signed [15:0] iq_q
);

    // ============================================================
    // TUNING WORD SHADOW
    // ============================================================
    reg [31:0] tuning_active;
    always @(posedge clk80) begin
        if (rst)
            tuning_active <= 32'd0;
        else if (tw_valid)
            tuning_active <= tuning_word;
    end

    // ============================================================
    // NCO
    // ============================================================
    wire signed [15:0] nco_sin;
    wire signed [15:0] nco_cos;

    nco_sincos_rx NCO (
        .clk        (clk80),
        .rst        (rst),
        .tuning_word(tuning_active),
        .sin_out    (nco_sin),
        .cos_out    (nco_cos)
    );

    // ============================================================
    // INLINE REGISTERED MULTIPLY
    // ============================================================

    // Stage 1: Register inputs (like INPUT_REG=1)
    reg signed [15:0] adc_r, sin_r, cos_r;
    always @(posedge clk80) begin
        if (rst) begin
            adc_r <= 0;
            sin_r <= 0;
            cos_r <= 0;
        end else begin
            adc_r <= {{4{adc_in[9]}}, adc_in, 2'b00};  // sign-extend + MSB-align 10-bit into the 12-bit slot
            sin_r <= nco_sin;
            cos_r <= nco_cos;
        end
    end

    // Stage 2: Multiplication (combinatorial)
    wire signed [31:0] mult_i_raw = $signed(adc_r) * $signed(cos_r);
    wire signed [31:0] mult_q_raw = $signed(adc_r) * $signed(sin_r);

    // Stage 3: Register outputs (like OUTPUT_REG=1)
    reg signed [31:0] mult_i_reg, mult_q_reg;
    always @(posedge clk80) begin
        if (rst) begin
            mult_i_reg <= 0;
            mult_q_reg <= 0;
        end else begin
            mult_i_reg <= mult_i_raw;
            mult_q_reg <= mult_q_raw;
        end
    end

    // ============================================================
    // SCALE MIXER OUTPUTS (Q15 -> Q2)  *** FIX 2 *** (+ FIX 4 narrowing, FIX 5)
    // ============================================================
    // A bare `>>> 13` is a floor, not a round: it removes on average half an
    // LSB from EVERY sample, which is a DC term, and the CIC then applies its
    // full x4.806 passband gain to it.
    //
    // *** FIX 5 *** shift is 13, not 15: the mixer output keeps two extra
    // fraction bits (x4), so its rounding noise is 12 dB lower relative to the
    // signal. Free, because the 10-bit sample has two zero LSBs in its 12-bit
    // slot. True range is now +/-8192 (2048 * 32767 >>> 13 = 8191.75).
    //
    // Same identity as the FIR: (x + 2^12) >>> 13  ==  x[31:13] + x[12], so
    // this is a 19-bit increment rather than a 32-bit add, and mixI/mixQ
    // shrink from 32 bits to 19 (14 bits + sign would do).
    localparam integer MIX_SHIFT = 13;
    localparam integer MIXSHW    = 32 - MIX_SHIFT;    // 19

    wire signed [MIXSHW-1:0] mi_hi = $signed(mult_i_reg[31:MIX_SHIFT]);
    wire signed [MIXSHW-1:0] mq_hi = $signed(mult_q_reg[31:MIX_SHIFT]);
    wire                     mi_rb = mult_i_reg[MIX_SHIFT-1];   // round bit
    wire                     mq_rb = mult_q_reg[MIX_SHIFT-1];

    reg signed [MIXSHW-1:0] mixI, mixQ;
    always @(posedge clk80) begin
        if (rst) begin
            mixI <= 0;
            mixQ <= 0;
        end else begin
            mixI <= mi_hi + $signed({{(MIXSHW-1){1'b0}}, mi_rb});
            mixQ <= mq_hi + $signed({{(MIXSHW-1){1'b0}}, mq_rb});
        end
    end

    // ============================================================
    // CIC Integrators                        *** FIX 1 ***
    // ============================================================
    // Widened 40 -> 48 bits so the integrators are the SAME width as the comb
    // chain below. A CIC decimator is only correct because the integrators'
    // modulo-2^W wraparound is exactly undone by the combs; if the combs are
    // wider, each wrap gets sign-extended into the wider arithmetic instead of
    // cancelling, and the difference taken across a wrap boundary produces a
    // huge wrong value. This is the bug found in simulation on the uDST -- the
    // plain 16-bit output slice masked it, and adding output saturation is
    // what made it visible.
    //
    // Width also has to cover the worst-case unwrapped range:
    //   |mix| <= 8192 (FIX 5), R^N = 432^3 = 80,621,568 -> 6.60e11
    //   -> 40 bits + sign.
    // 48 bits is comfortably above the 41-bit minimum, and matching the combs
    // is the property that actually matters.
    localparam integer CICW = 48;
    reg signed [CICW-1:0] i1I, i2I, i3I;
    reg signed [CICW-1:0] i1Q, i2Q, i3Q;
    always @(posedge clk80) begin
        if (rst) begin
            i1I <= 0; i2I <= 0; i3I <= 0;
            i1Q <= 0; i2Q <= 0; i3Q <= 0;
        end else begin
            i1I <= i1I + mixI;
            i2I <= i2I + i1I;
            i3I <= i3I + i2I;
            i1Q <= i1Q + mixQ;
            i2Q <= i2Q + i1Q;
            i3Q <= i3Q + i2Q;
        end
    end

    // ============================================================
    // Decimation control
    // ============================================================
    reg [8:0] dec_cnt;
    always @(posedge clk80)
        if (rst)                    dec_cnt <= 0;
        else if (dec_cnt == 9'd431) dec_cnt <= 0;   // decimate-by-432
        else                        dec_cnt <= dec_cnt + 1'b1;

    wire dec_tick = (dec_cnt == 9'd431);

    // *** FIX 4 *** one-clock-delayed tick that fires the second half of the
    // output stage. Costs 1 clock out of the 432 available per output sample.
    reg dec_tick_d1;
    always @(posedge clk80)
        if (rst) dec_tick_d1 <= 1'b0;
        else     dec_tick_d1 <= dec_tick;

    reg               cic_valid_reg;
    reg signed [15:0] cicI_reg, cicQ_reg;   // saturated 16-bit CIC output

    // ============================================================
    // CIC Combs (pipelined to make comb result stable before valid)
    // ============================================================
    reg signed [CICW-1:0] i3I_s, i3Q_s;                  // sampled integrators

    reg signed [CICW-1:0] comb_d1I, comb_d2I, comb_d3I;
    reg signed [CICW-1:0] comb_d1Q, comb_d2Q, comb_d3Q;

    reg signed [CICW-1:0] y0I, y1I, y2I;
    reg signed [CICW-1:0] y0Q, y1Q, y2Q;

    // ------------------------------------------------------------
    // *** FIX 3 + FIX 4 *** CIC output: round/shift, then saturate/slice,
    // split across TWO clocks.
    // ------------------------------------------------------------
    // *** FIX 5 *** >>>24 gives 432^3/2^24 = x4.806 (432 is not a power of
    // two). With the mixer's x4 and the RX FIR's x1.665 the overall RX gain is
    // x32.0, matching the 80 MHz uDST at CIC >>>22.
    //
    //   Stage E1 (on dec_tick):    y2 -> 24-bit rounded/shifted value
    //   Stage E2 (on dec_tick_d1): saturate to 16 bits, assert cic_valid
    //
    // Splitting here is what recovers Fmax: the old single-clock version was a
    // 48-bit adder + 33-bit reduction + mux, 10 logic levels deep, and it was
    // the sole source of every setup violation in the design.
    //
    // A full-scale ADC sine gives a CIC output of +/-19684 (4096 * 4.806), so
    // the saturation should never fire in normal operation. The only way to
    // reach it is a near-full-scale DC level at the ADC with the NCO at 0 Hz
    // (8192 * 4.806 = 39370), which the transformer-coupled front end cannot
    // produce. It remains the safety net that stops a wrap from turning a
    // strong signal into full-scale noise, and it is what will expose any
    // remaining arithmetic problem instead of hiding it.
    localparam integer CIC_SHIFT = 24;
    localparam integer CICSHW    = CICW - CIC_SHIFT;   // 24 bits after the shift

    wire signed [CICSHW-1:0] y2I_hi = $signed(y2I[CICW-1:CIC_SHIFT]);
    wire signed [CICSHW-1:0] y2Q_hi = $signed(y2Q[CICW-1:CIC_SHIFT]);
    wire                     y2I_rb = y2I[CIC_SHIFT-1];   // round bit
    wire                     y2Q_rb = y2Q[CIC_SHIFT-1];

    reg signed [CICSHW-1:0] y2I_sh, y2Q_sh;   // rounded AND shifted (stage E1)

    // "fits in signed 16 bits" == every bit above bit 15 equals the sign bit.
    // 9 bits to test ([23:15]), not 33: after the >>>24 the old test's upper
    // bits were sign copies of each other and carried no information.
    wire y2I_fit = (&y2I_sh[CICSHW-1:15]) | (~|y2I_sh[CICSHW-1:15]);
    wire y2Q_fit = (&y2Q_sh[CICSHW-1:15]) | (~|y2Q_sh[CICSHW-1:15]);

    wire signed [15:0] cicI_sat = y2I_fit ? y2I_sh[15:0]
                                          : (y2I_sh[CICSHW-1] ? -16'sd32768 : 16'sd32767);
    wire signed [15:0] cicQ_sat = y2Q_fit ? y2Q_sh[15:0]
                                          : (y2Q_sh[CICSHW-1] ? -16'sd32768 : 16'sd32767);

    always @(posedge clk80) begin
        if (rst) begin
            i3I_s <= 0; i3Q_s <= 0;
            comb_d1I <= 0; comb_d2I <= 0; comb_d3I <= 0;
            comb_d1Q <= 0; comb_d2Q <= 0; comb_d3Q <= 0;
            y0I <= 0; y1I <= 0; y2I <= 0;
            y0Q <= 0; y1Q <= 0; y2Q <= 0;
            y2I_sh <= 0; y2Q_sh <= 0;
            cicI_reg <= 0; cicQ_reg <= 0;
            cic_valid_reg <= 1'b0;
        end else begin
            cic_valid_reg <= 1'b0;   // default: deassert valid

            if (dec_tick) begin
                // Stage A: sample integrator outputs at decimation instant
                i3I_s <= i3I;
                i3Q_s <= i3Q;

                // Stage B: first comb difference, y0 = x[n] - x[n-1]
                y0I <= i3I_s - comb_d1I;
                y0Q <= i3Q_s - comb_d1Q;

                comb_d1I <= i3I_s;
                comb_d1Q <= i3Q_s;

                // Stage C: second comb difference
                y1I <= y0I - comb_d2I;
                y1Q <= y0Q - comb_d2Q;

                comb_d2I <= y0I;
                comb_d2Q <= y0Q;

                // Stage D: third comb difference
                y2I <= y1I - comb_d3I;
                y2Q <= y1Q - comb_d3Q;

                comb_d3I <= y1I;
                comb_d3Q <= y1Q;

                // Stage E1: round + shift only (24-bit increment)
                y2I_sh <= y2I_hi + $signed({{(CICSHW-1){1'b0}}, y2I_rb});
                y2Q_sh <= y2Q_hi + $signed({{(CICSHW-1){1'b0}}, y2Q_rb});
            end

            if (dec_tick_d1) begin
                // Stage E2: saturate + slice, then assert valid
                cicI_reg <= cicI_sat;
                cicQ_reg <= cicQ_sat;
                cic_valid_reg <= 1'b1;
            end
        end
    end

    // *** FIX 4 *** the old extra cicI/cicQ register pair is gone. It used to
    // reload one clock AFTER cic_valid went high, which meant the FIR captured
    // the PREVIOUS decimated sample -- harmless (a constant one-sample delay)
    // but confusing. cicI_reg/cicQ_reg are already registered and are now
    // written on the same clock that sets cic_valid_reg, so the FIR sees the
    // matching sample and 32 flip-flops come back.
    wire cic_valid = cic_valid_reg;

    // FIR Decimator
    wire fir_valid;
    wire signed [15:0] firI, firQ;
    fir252_dec6_rx FIR (
        .clk        (clk80),
        .rst        (rst),
        .din_valid  (cic_valid),
        .xi         (cicI_reg),
        .xq         (cicQ_reg),
        .yoI        (firI),
        .yoQ        (firQ),
        .dout_valid (fir_valid)
    );

    assign iq_valid = fir_valid;
    assign iq_i     = firI;
    assign iq_q     = firQ;

endmodule
