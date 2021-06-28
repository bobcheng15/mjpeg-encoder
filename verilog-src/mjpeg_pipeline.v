module mjpeg_pipeline(
    input clk,
    input rst_n,
    input [2:0] component_row_index,
    input [2:0] component_column_index,
    input [2:0] data_row_index,
    input [2:0] data_column_index,
    input [7:0] input_pixel,
    output reg signed [31:0] result
);
// pipeline buffer for stage rad_cal/rad_proc
reg [7:0] input_pixel_rad_cal_proc;
reg [6:0] cosine_rad_row, next_cosine_rad_row;
reg [6:0] cosine_rad_column, next_cosine_rad_column;
// pipeline buffer for stage rad_proc/cosine
reg [7:0] input_pixel_rad_proc_cosine;
reg [3:0] proc_cosine_rad_row, next_proc_cosine_rad_row;
reg [3:0] proc_cosine_rad_column, next_proc_cosine_rad_column;
reg invert_row, invert_column;
reg next_invert_row, next_invert_column;
// pipeline buffer for stage cosine/result
reg [7:0] input_pixel_cosine_result;
reg signed [7:0] cosine_row, cosine_column;
reg signed [7:0] next_cosine_row, next_cosine_column;
wire signed [7:0] cosine_lookup_row, cosine_lookup_column;
reg signed [31:0] next_result;
// cosine lookup table for row
cosine_lut cos_lut_row(
    .rad(proc_cosine_rad_row),
    .cosine(cosine_lookup_row)
);
// cosine lookup table for column
cosine_lut cos_lut_column(
    .rad(proc_cosine_rad_column),
    .cosine(cosine_lookup_column)
);
always@(posedge clk) begin
    // reset all buffer on reset
    if (!rst_n) begin
        input_pixel_rad_cal_proc <= 8'd0;
        input_pixel_rad_proc_cosine <= 8'd0;
        input_pixel_cosine_result <= 8'd0;
        cosine_rad_row <= 7'd0;
        cosine_rad_column <= 7'd0;
        proc_cosine_rad_row <= 4'd0;
        proc_cosine_rad_column <= 4'd0;
        cosine_row <= 8'd0;
        cosine_column <= 8'd0;
        result <= 32'd0;
        invert_row <= 1'd0;
        invert_column <= 1'd0;
    end
    else begin
        input_pixel_rad_cal_proc <= input_pixel;
        input_pixel_rad_proc_cosine <= input_pixel_rad_cal_proc;
        input_pixel_cosine_result <= input_pixel_rad_proc_cosine;
        cosine_rad_row <= next_cosine_rad_row;
        cosine_rad_column <= next_cosine_rad_column;
        proc_cosine_rad_row <= next_proc_cosine_rad_row;
        proc_cosine_rad_column <= next_proc_cosine_rad_column;
        cosine_row <= next_cosine_row;
        cosine_column <= next_cosine_column;
        result <= next_result;
    end
end

always@(*) begin
    // stage rad_cal
    next_cosine_rad_row = ((data_row_index << 2) + 1) * component_row_index;
    next_cosine_rad_column = ((data_column_index << 2) + 1) * component_column_index;
    // stage rad_proc
    // row
    if (cosine_rad_row[4:0] > 24) begin
        next_proc_cosine_rad_row = 6'd32 - cosine_rad_row[4:0];
        next_invert_row = 1'd0;
    end
    else if (cosine_rad_row[4:0] > 16) begin
        next_proc_cosine_rad_row = 6'd16 - cosine_rad_row[4:0];
        next_invert_row = 1'd1;
    end
    else if (cosine_rad_row[4:0] > 8) begin
        next_proc_cosine_rad_row = 6'd8 - cosine_rad_row[4:0];
        next_invert_row = 1'd1;
    end
    else begin
        next_proc_cosine_rad_row = cosine_rad_row[4:0];
        next_invert_row = 1'd0;
    end
    // column
    if (cosine_rad_column[4:0] > 24) begin
        next_proc_cosine_rad_column = 6'd32 - cosine_rad_column[4:0];
        next_invert_column = 1'd0;
    end
    else if (cosine_rad_column[4:0] > 16) begin
        next_proc_cosine_rad_column = 6'd16 - cosine_rad_column[4:0];
        next_invert_column = 1'd1;
    end
    else if (cosine_rad_column[4:0] > 8) begin
        next_proc_cosine_rad_column = 6'd8 - cosine_rad_column[4:0];
        next_invert_column = 1'd1;
    end
    else begin
        next_proc_cosine_rad_column = cosine_rad_column[4:0];
        next_invert_column = 1'd0;
    end
    // stage cosine
    next_cosine_row = (invert_row)? cosine_lookup_row * -1: cosine_lookup_row;
    next_cosine_column = (invert_column)? cosine_lookup_column * -1: cosine_lookup_column;
    // stage result
    next_result = cosine_row * cosine_column;
end

endmodule


module cosine_lut(
    input [3:0] rad,
    output reg [7:0] cosine
);

always@(*) begin
    case(rad)
        4'd0: cosine = 8'd100;
        4'd1: cosine = 8'd98;
        4'd2: cosine = 8'd92;
        4'd3: cosine = 8'd83;
        4'd4: cosine = 8'd71;
        4'd5: cosine = 8'd56;
        4'd6: cosine = 8'd38;
        4'd7: cosine = 8'd20;
        4'd8: cosine = 8'd0;
    endcase
end


endmodule
