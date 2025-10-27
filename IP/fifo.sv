// Generic synchronous FIFO (renamed from fifo_generic)
module fifo#(
	parameter DATA_WIDTH = 32,
	parameter DEPTH = 16
)(
	input  logic clk,
	input  logic reset,
	input  logic wr_en,
	input  logic [DATA_WIDTH-1:0] wr_data,
	input  logic rd_en,
	output logic [DATA_WIDTH-1:0] rd_data,
	output logic full,
	output logic empty
);
	localparam ADDR_WIDTH = $clog2(DEPTH);
	
	logic [DATA_WIDTH-1:0] memory [DEPTH];
	logic [ADDR_WIDTH:0] wr_ptr, rd_ptr; // Extra bit for full/empty detection
	logic [ADDR_WIDTH:0] count;
	// Write operation
	always_ff @(posedge clk or posedge reset) begin
		if (reset) begin
			wr_ptr <= '0;
		end else if (wr_en && !full) begin
			memory[wr_ptr[ADDR_WIDTH-1:0]] <= wr_data;
			wr_ptr <= wr_ptr + 1;
		end
	end
	
	// Read operation
	always_ff @(posedge clk or posedge reset) begin
		if (reset) begin
			rd_ptr <= '0;
			rd_data <= '0;
		end else if (rd_en && !empty) begin
			rd_data <= memory[rd_ptr[ADDR_WIDTH-1:0]];
			rd_ptr <= rd_ptr + 1;
		end
	end
	
	// Count management
	always_ff @(posedge clk or posedge reset) begin
		if (reset) begin
			count <= '0;
		end else begin
			case ({wr_en && !full, rd_en && !empty})
				2'b10: count <= count + 1;
				// Write only
				2'b01: count <= count - 1;  // Read only
				default: count <= count;
				// No change or both
			endcase
		end
	end
	
	// Status flags
	assign full = (count == DEPTH);
	assign empty = (count == 0);

endmodule