// Imported from Micah Elizabeth Scott
// Additinoal re-work, pixel accurate timing and
// optimizations (C) 2020 René Rebe <rene@exactcode.de>

// 640x480 26MHz 60Hz, +sync

`define h_fp        16
`define h_sync      64
`define h_bp        80
`define h_active    640
`define v_fp        3
`define v_sync      4
`define v_bp        16
`define v_active    480


`define state_fp        0
`define state_sync      1
`define state_bp        2
`define state_active    3
 
module video_timing (
    input clk,
    output hsync,
    output vsync,
    output data_en,
    output [15:0] xpos,
    output [15:0] ypos,
);
   
    ////////////////////////////
    // horizontal

    // | fp | sync | bp | active |
   
    reg signed [15:0] h_cnt = 0;
    reg [3:0] 	     h_state = 8;
    wire [3:0] 	     h_state_next = { h_state[2:0], h_state[3] };
   
    assign xpos = h_cnt;

    always @(posedge clk) begin
       h_cnt <= h_cnt + 1;
       
       case (h_cnt)
	 16'd0 - `h_sync - `h_bp - 1:
	   h_state <= h_state_next;
	 16'd0 - `h_bp - 1:
	   h_state <= h_state_next;
	 16'd0 - 1:
	   h_state <= h_state_next;
	 16'd0 + `h_active - 1:
	   begin
	    h_state <= h_state_next;
	      h_cnt <= 16'd0 - `h_fp - `h_sync - `h_bp;
	   end
       endcase
    end

    ////////////////////////////
    // vertical

    reg [3:0] h_state_prev;

    reg signed [15:0] v_cnt = 0;
    reg [3:0] 	     v_state = 8;
    wire [3:0] 	     v_state_next = { v_state[2:0], v_state[3] };

    assign ypos = v_cnt;

    reg [3:0]   h_state_prev;
    wire        h_rollover =  h_state[`state_sync] & h_state_prev[`state_fp];

    always @(posedge clk) begin
        h_state_prev <= h_state;
    end

    always @(posedge clk) begin
       if (h_rollover) begin
	  v_cnt <= v_cnt + 1;
	  
	  case (v_cnt)
	    16'd0 - `v_sync - `v_bp - 1:
	      v_state <= v_state_next;
	    16'd0 - `v_bp - 1:
	      v_state <= v_state_next;
	    16'd0 - 1:
	      v_state <= v_state_next;
	    16'd0 + `v_active - 1:
	      begin
		 v_state <= v_state_next;
		 v_cnt <= 16'd0 - `v_fp - `v_sync - `v_bp;
	      end
	  endcase
       end
    end

    ////////////////////////////
    // output

    assign hsync = h_state[`state_sync];
    assign vsync = v_state[`state_sync];
    assign data_en = h_state[`state_active] & v_state[`state_active];

endmodule
