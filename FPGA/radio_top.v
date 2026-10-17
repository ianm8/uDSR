// ============================================================================
// radio_top -- direct-sampling HF receiver, top level.  *** RX-ONLY BUILD ***
//
//   81 MHz variant (native 27 MHz crystal, x3 = 81 MHz), 10-BIT ADC,
//   SINGLE SPI bus carrying IQ out and the tuning word in.
//
// Derived from the 81 MHz transceiver top. Changes in this build:
//
//   1. RX only. DUC, DAC bus (10 pins), tx_rx pin, mode_tx logic and the
//      DAC output register are all removed. DUC18.v is NO LONGER a project
//      source; mac16_reg (shared by the RX FIR) lives in DDC22.v.
//
//   2. 10-bit ADC (e.g. AD9215-105) instead of the 12-bit AD9236. Inside
//      ddc_top the sample is MSB-aligned into the old 12-bit position
//      ({adc,2'b00}), so every gain number -- CIC >>>23, FIR x1.665,
//      overall x16 -- and the Pico-side IQ scaling are IDENTICAL to the
//      transceiver build. Only the quantisation step changes (4 counts).
//
//   3. The write-only tuning-word SPI bus is GONE (3 pins freed). The IQ
//      SPI bus is now full duplex: while the Pico clocks the 32-bit {I,Q}
//      sample out on MISO it clocks a 32-bit candidate tuning word in on
//      MOSI. At the end of the frame (CS rising edge) the FPGA accepts the
//      word only if it is NON-ZERO and the frame contained exactly 32 SCK
//      rising edges; a zero word, or a malformed frame, is ignored.
//        Pico rule: send the tuning word in the frame after it changes,
//        0x00000000 otherwise. The Pico's SPI is full duplex anyway
//        (spi_write_read_blocking), so this is one transfer per sample tick
//        instead of two buses. Re-sending the same word is harmless: the NCO
//        phase accumulator free-runs and re-latching an identical word
//        causes no phase step, so no click. NOTE the Pico MUST send the real
//        word at least once after FPGA reset (tuning_word resets to 0 and
//        the NCO sits at DC until then) -- it would anyway on boot.
//        tw = round(f * 2^32 / 81e6) is never 0 for any real dial frequency,
//        so 0 is a safe "no change" sentinel.
//
// Sample chain: 81 MHz -> CIC /432 -> 187.5 kHz -> FIR /6 -> 31.25 kHz
// (total /2592). Internal clock nets keep the clk80 names but carry 81 MHz.
// The Pico computes NCO tuning words with f_clk = 81e6.
//
// SPI: mode 0 (CPOL=0, CPHA=0), 32-bit frames, MSB first, CS active low,
// oversampled in the clk80 domain -- keep SCK to a few MHz so a bit period
// spans several 81 MHz cycles, and leave a few clk80 cycles between CS
// falling and the first SCK edge. MOSI is board-pulled DOWN (see .cst): an
// undriven MOSI shifts in zeros = "no change", which is fail-safe.
//
// Pin budget (16 of 32 GPIO; sys_clk on the crystal pin):
//   adc 10 + clk80 1 + spi_iq 4 + data_ready 1 = 16.   16 GPIO now FREE
//   (the old adc[1:0], dac[9:0], tx_rx, spi_tw sck/mosi/cs_n).
// leds_n[1:6] stay on the dedicated 1.8V LED header.
//
// Project sources: this file, DDC22.v (10-bit port variant), pll_27_81.v,
// sin_lut.hex, fir_rx252_coeffs.hex. ODDR is a Gowin primitive. The rPLL IP
// (27 MHz in -> 81 MHz out: IDIV /1, FBDIV x3, ODIV 8, VCO 648 MHz) is
// unchanged from the 81 MHz transceiver build.
// ============================================================================
module radio_top (
    input  wire        sys_clk,        // 27 MHz crystal (clock pin)

    input  wire signed [9:0] adc_in,   // 10-bit ADC, two's complement

    output wire        clk80,          // 81 MHz forwarded to the ADC (net name kept for .cst)

    input  wire        spi_iq_sck,
    input  wire        spi_iq_mosi,    // tuning word in (0 = no change)
    output wire        spi_iq_miso,    // {I[15:0],Q[15:0]} sample out
    input  wire        spi_iq_cs_n,

    output reg         data_ready,
    output wire [1:6]  leds_n          // active low, 1.8V bank; status scroller
);

    // ---- clocking: 27 MHz -> rPLL -> 81 MHz ----
    wire clk80_int, pll_lock;
    pll_27_81 u_pll (.clkin(sys_clk), .clkout(clk80_int), .lock(pll_lock));

    // ---- internal reset (no pin): hold until lock, sync de-assert ----
    reg [1:0] rst_pipe = 2'b11;
    always @(posedge clk80_int or negedge pll_lock) begin
        if (!pll_lock) rst_pipe <= 2'b11;
        else           rst_pipe <= {rst_pipe[0], 1'b0};
    end
    wire rst = rst_pipe[1];

    // ---- forward clk80 to the ADC via DDR output buffer ----
    ODDR u_clk_fwd (.D0(1'b1), .D1(1'b0), .TX(1'b0), .CLK(clk80_int), .Q0(clk80), .Q1());

    // ---- ADC capture (IOB register) ----
    reg signed [9:0] adc_r;
    always @(posedge clk80_int) adc_r <= adc_in;

    // ---- combined IQ + tuning-word SPI slave ----
    wire [31:0] tuning_word;
    wire        tw_valid;
    wire               rx_iq_valid;
    wire signed [15:0] rx_iq_i, rx_iq_q;
    spi_iq_tw_slave u_iq (
        .clk(clk80_int), .rst(rst),
        .sck(spi_iq_sck), .mosi(spi_iq_mosi), .cs_n(spi_iq_cs_n), .miso(spi_iq_miso),
        .rx_i(rx_iq_i), .rx_q(rx_iq_q), .rx_valid(rx_iq_valid),
        .tuning_word(tuning_word), .tw_valid(tw_valid)
    );

    // ---- RX chain: ADC -> DDC -> IQ ----
    ddc_top u_ddc (
        .clk80(clk80_int), .rst(rst), .adc_in(adc_r),
        .tw_valid(tw_valid), .tuning_word(tuning_word),
        .iq_valid(rx_iq_valid), .iq_i(rx_iq_i), .iq_q(rx_iq_q)
    );

    // ---- data_ready: 50% square wave phase-locked to the DDC sample tick ----
    //   high on iq_valid, low after 1296 cycles, until the next iq_valid.
    //   1296 = (81e6/31250)/2 = 2592/2. Pico waits high, runs DSP, waits low.
    localparam [11:0] HB_HALF = 12'd1296;
    reg [11:0] dr_cnt = 12'd0;
    always @(posedge clk80_int) begin
        if (rst) begin
            data_ready <= 1'b0; dr_cnt <= 12'd0;
        end else if (rx_iq_valid) begin
            data_ready <= 1'b1; dr_cnt <= 12'd0;
        end else if (data_ready) begin
            if (dr_cnt == HB_HALF-1) data_ready <= 1'b0;
            else                     dr_cnt <= dr_cnt + 12'd1;
        end
    end

    // ---- status LEDs: single LED scrolling around the ring of 6.
    //   RX only now: step every 2^25 clk80 cycles = 2.414 Hz (~414 ms @81 MHz)
    reg [24:0] led_div;
    always @(posedge clk80_int) begin
        if (rst) led_div <= 25'd0;
        else     led_div <= led_div + 25'd1;
    end
    wire led_tick = (led_div[24:0] == 25'd0);

    reg [2:0] led_pos;     // 0..5, wraps
    always @(posedge clk80_int) begin
        if (rst)            led_pos <= 3'd0;
        else if (led_tick)  led_pos <= (led_pos == 3'd5) ? 3'd0 : led_pos + 3'd1;
    end

    reg [1:6] leds_n_r;
    always @(posedge clk80_int) begin
        if (rst) leds_n_r <= 6'b111111;
        else case (led_pos)
            3'd0: leds_n_r <= 6'b111110;
            3'd1: leds_n_r <= 6'b111101;
            3'd2: leds_n_r <= 6'b111011;
            3'd3: leds_n_r <= 6'b110111;
            3'd4: leds_n_r <= 6'b101111;
            default: leds_n_r <= 6'b011111;   // 3'd5
        endcase
    end
    assign leds_n = leds_n_r;

endmodule


// ============================================================================
// spi_iq_tw_slave -- full-duplex SPI slave (mode 0, MSB first, 32-bit frame).
//
// MISO: on rx_valid latch the DDC sample into rx_hold; on CS falling load it
// into the output shifter and stream {I[15:0],Q[15:0]} out (advance on SCK
// falling edges, master samples on rising -- classic mode 0).
//
// MOSI: shift in on SCK rising edges. On CS rising, the received word is
// accepted as a new tuning word ONLY IF it is non-zero AND the frame
// contained exactly 32 SCK rising edges (bit_cnt saturates at 63, so over-
// long frames also fail the ==32 test). A zero word means "no change"; a
// malformed frame is discarded rather than retuning the radio to garbage --
// stricter than the old spi_tw_slave, which would have silently latched a
// truncated word.
// ============================================================================
module spi_iq_tw_slave (
    input  wire clk, input wire rst,
    input  wire sck, input wire mosi, input wire cs_n,
    output reg  miso,
    input  wire signed [15:0] rx_i, input wire signed [15:0] rx_q,
    input  wire rx_valid,
    output reg  [31:0] tuning_word,
    output reg         tw_valid
);
    // ---- input synchronisers, 3-stage ----
    // Deliberately NOT on rst: synchroniser chains self-flush in 3 clocks and
    // rst is held until PLL lock, long after. Gowin fabric FFs load their
    // INIT value at configuration, so power-up state is defined: cs_s starts
    // 3'b111 to match the idle (pulled-up) CS line, so the flush cannot
    // manufacture a false cs_rise; sck/mosi idle low, matching the 0 default.
    reg [2:0] sck_s  = 3'b000;
    reg [2:0] cs_s   = 3'b111;
    reg [1:0] mosi_s = 2'b00;
    always @(posedge clk) begin
        sck_s  <= {sck_s[1:0],  sck};
        cs_s   <= {cs_s[1:0],   cs_n};
        mosi_s <= {mosi_s[0],   mosi};
    end
    // Edge detects in dedicated flops (Fmax fix), decoded from the FULLY
    // SYNCHRONISED taps [2:1] -- never from stage [0]. Stage [0] is the
    // metastability-capture flop and must fan out to exactly one load
    // (sck_s[1]/cs_s[1]); decoding from [1:0] would let one metastable event
    // resolve differently at different flops, giving double or missed pulses.
    // All five signals are uniformly one clk80 cycle later than the old
    // combinational decodes, so their relative alignment is unchanged, e.g.
    // cs_fall(k) still implies cs_low(k) = ~cs_s[2](k) = 1 the same cycle.
    // MOSI is sampled ~one clk80 later into the held bit: harmless at SPI
    // rates and the safe direction for mode 0. Pulse flops need no reset:
    // they settle one cycle after the synchronisers and the FSM holds in rst
    // until PLL lock anyway.
    reg sck_rise = 1'b0, sck_fall = 1'b0, cs_fall = 1'b0, cs_rise = 1'b0;
    reg cs_low = 1'b0;
    always @(posedge clk) begin
        sck_rise <= (sck_s[2:1] == 2'b01);
        sck_fall <= (sck_s[2:1] == 2'b10);
        cs_low   <= ~cs_s[1];
        cs_fall  <= (cs_s[2:1] == 2'b10);   // frame start
        cs_rise  <= (cs_s[2:1] == 2'b01);   // frame end
    end

    // latest DDC sample, held for the next read
    reg [31:0] rx_hold;
    always @(posedge clk) begin
        if (rst) rx_hold <= 0;
        else if (rx_valid) rx_hold <= {rx_i, rx_q};
    end

    reg [31:0] sr_out;    // -> MISO ({I,Q} sample)
    reg [31:0] sr_in;     // <- MOSI (candidate tuning word)
    reg [5:0]  bit_cnt;   // SCK rising edges this frame, saturating at 63

    always @(posedge clk) begin
        if (rst) begin
            sr_out <= 0; sr_in <= 0; bit_cnt <= 0; miso <= 1'b0;
            tuning_word <= 32'd0; tw_valid <= 1'b0;
        end else begin
            tw_valid <= 1'b0;
            if (cs_fall) begin
                sr_out  <= rx_hold;                  // load word to send
                miso    <= rx_hold[31];
                sr_in   <= 32'd0;
                bit_cnt <= 6'd0;
            end
            if (cs_low && sck_rise) begin
                sr_in <= {sr_in[30:0], mosi_s[1]};   // shift tuning word in
                if (bit_cnt != 6'd63) bit_cnt <= bit_cnt + 6'd1;
            end
            if (cs_low && sck_fall) begin
                sr_out <= {sr_out[30:0], 1'b0};      // advance, present next bit
                miso   <= sr_out[30];
            end
            if (cs_rise) begin
                if ((bit_cnt == 6'd32) && (sr_in != 32'd0)) begin
                    tuning_word <= sr_in;
                    tw_valid    <= 1'b1;
                end
            end
        end
    end
endmodule
