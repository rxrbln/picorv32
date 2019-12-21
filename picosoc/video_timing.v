// Normal standard 720p at 60hz
// 1280x720 74.25MHz 60Hz, +sync (HDMI modes 4,69)
/*
`define h_fp        110
`define h_sync      40
`define h_bp        220
`define h_active    1280
`define v_fp        5
`define v_sync      5
`define v_bp        20
`define v_active    720
*/

// 1920x1080 74.25MHz 30Hz, +sync (HDMI modes 34,74)
// This has a lower vertical refresh rate than some monitors will tolerate
/*
`define h_fp        88
`define h_sync      44
`define h_bp        148
`define h_active    1920
`define v_fp        4
`define v_sync      5
`define v_bp        36
`define v_active    1080
*/

`define h_fp        16
`define h_sync      96
`define h_bp        48
`define h_active    640
`define v_fp        10
`define v_sync      2
`define v_bp        33
`define v_active    480


`define h_ctr_bits  $clog2(`h_active)
`define v_ctr_bits  $clog2(`v_active)

`define state_fp        0
`define state_sync      1
`define state_bp        2
`define state_active    3

module video_timing (
    input clk,
    output hsync,
    output vsync,
    output data_en
);

    ////////////////////////////
    // horizontal

    reg [`h_ctr_bits:0] h_ctr = 0;
    reg [3:0] h_state = 1;
    wire [3:0] h_state_next = { h_state[2:0], h_state[3] };

    always @(posedge clk) begin
        if (h_ctr[`h_ctr_bits]) begin
            h_state <= h_state_next;
            h_ctr <=
                h_state_next[`state_fp] ? `h_fp - 2 :
                h_state_next[`state_sync] ? `h_sync - 2 :
                h_state_next[`state_bp] ? `h_bp - 2 :
                h_state_next[`state_active] ? `h_active - 2 :
                16'hXXXX;
        end
        else
            h_ctr <= h_ctr - 1;
    end

    ////////////////////////////
    // vertical

    reg [3:0] h_state_prev;
    reg h_rollover;
    always @(posedge clk) begin
        h_state_prev <= h_state;
        h_rollover <= h_state[`state_sync] & h_state_prev[`state_fp];
    end

    reg [`v_ctr_bits:0] v_ctr = 0;
    reg [3:0] v_state = 1;
    wire [3:0] v_state_next = { v_state[2:0], v_state[3] };

    always @(posedge clk) begin
        if (h_rollover) begin
            if (v_ctr[`v_ctr_bits]) begin
                v_state <= v_state_next;
                v_ctr <=
                    v_state_next[`state_fp] ? `v_fp - 2 :
                    v_state_next[`state_sync] ? `v_sync - 2 :
                    v_state_next[`state_bp] ? `v_bp - 2 :
                    v_state_next[`state_active] ? `v_active - 2 :
                    16'hXXXX;
            end
            else
                v_ctr <= v_ctr - 1;
        end
    end

    ////////////////////////////
    // output

    reg hsync;
    reg vsync;
    reg data_en;

    always @(posedge clk) begin
        hsync <= h_state[`state_sync];
        vsync <= v_state[`state_sync];
        data_en <= h_state[`state_active] & v_state[`state_active];
    end

endmodule
