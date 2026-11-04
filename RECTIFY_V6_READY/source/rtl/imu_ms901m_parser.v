`timescale 1ns / 1ps
// Parser for the ATK-MS901M active-report protocol.
// Frame: 55 55 ID LEN DATA[LEN] SUM, with an 8-bit wrapping sum.
module imu_ms901m_parser #(parameter integer TIMEOUT_CYCLES=270000) (
    input                    clk,
    input                    rst_n,
    input      [7:0]         rx_data,
    input                    rx_valid,
    output reg               attitude_valid,
    output reg signed [15:0] roll_raw,
    output reg signed [15:0] pitch_raw,
    output reg signed [15:0] yaw_raw,
    output reg quaternion_valid,magnetic_valid,
    output reg signed [15:0] q0_raw,q1_raw,q2_raw,q3_raw,mx_raw,my_raw,mz_raw,mag_temperature_raw,
    output reg               motion_valid,
    output reg signed [15:0] ax_raw,
    output reg signed [15:0] ay_raw,
    output reg signed [15:0] az_raw,
    output reg signed [15:0] gx_raw,
    output reg signed [15:0] gy_raw,
    output reg signed [15:0] gz_raw,
    output reg               altitude_valid,
    output reg signed [31:0] pressure_raw,
    output reg signed [31:0] altitude_cm,
    output reg signed [15:0] temperature_raw,
    output reg [7:0]         frame_id,
    output reg               frame_error,
    output reg [1:0]          gyro_range,acc_range,
    output reg               gyro_known,acc_known,algorithm_known,algorithm_mag,
    output reg               mounting_known,mounting_vertical
);
    localparam [2:0] P_H1 = 3'd0;
    localparam [2:0] P_H2 = 3'd1;
    localparam [2:0] P_ID = 3'd2;
    localparam [2:0] P_LEN = 3'd3;
    localparam [2:0] P_DATA = 3'd4;
    localparam [2:0] P_SUM = 3'd5;

    reg [2:0] state;
    reg [31:0] idle_count;
    reg [7:0] data_len;
    reg [7:0] data_count;
    reg [7:0] sum;
    reg response_header;
    reg [7:0] data0, data1, data2, data3, data4, data5;
    reg [7:0] data6, data7, data8, data9, data10, data11;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= P_H1;idle_count<=0;
            response_header<=0;gyro_range<=0;acc_range<=0;
            gyro_known<=0;acc_known<=0;algorithm_known<=0;algorithm_mag<=0;
            mounting_known<=0;mounting_vertical<=0;
            data_len <= 0;
            data_count <= 0;
            sum <= 0;
            data0 <= 0; data1 <= 0; data2 <= 0; data3 <= 0;
            data4 <= 0; data5 <= 0; data6 <= 0; data7 <= 0;
            data8 <= 0; data9 <= 0; data10 <= 0; data11 <= 0;
            attitude_valid <= 1'b0;
            quaternion_valid<=0;magnetic_valid<=0;q0_raw<=0;q1_raw<=0;q2_raw<=0;q3_raw<=0;
            mx_raw<=0;my_raw<=0;mz_raw<=0;mag_temperature_raw<=0;
            motion_valid <= 1'b0;
            altitude_valid <= 1'b0;
            frame_error <= 1'b0;
            frame_id <= 0;
            roll_raw <= 0; pitch_raw <= 0; yaw_raw <= 0;
            ax_raw <= 0; ay_raw <= 0; az_raw <= 0;
            gx_raw <= 0; gy_raw <= 0; gz_raw <= 0;
            pressure_raw <= 0; altitude_cm <= 0; temperature_raw <= 0;
        end else begin
            quaternion_valid<=0;magnetic_valid<=0;
            attitude_valid <= 1'b0;
            motion_valid <= 1'b0;
            altitude_valid <= 1'b0;
            frame_error <= 1'b0;

            if(rx_valid || state==P_H1) idle_count<=0;
            else if(idle_count>=TIMEOUT_CYCLES-1) begin state<=P_H1;idle_count<=0;frame_error<=1;end
            else idle_count<=idle_count+1'b1;
            if (rx_valid) begin
                case (state)
                    P_H1: begin
                        if (rx_data == 8'h55)
                            state <= P_H2;
                    end

                    P_H2: begin
                        if (rx_data == 8'h55 || rx_data == 8'hAF) begin
                            response_header<=(rx_data==8'hAF);
                            state <= P_ID;
                        end else
                            state <= P_H1;
                    end

                    P_ID: begin
                        frame_id <= rx_data;
                        sum <= (response_header?8'h04:8'hAA) + rx_data;
                        state <= P_LEN;
                    end

                    P_LEN: begin
                        data_len <= rx_data;
                        data_count <= 0;
                        sum <= sum + rx_data;
                        if ((rx_data != 0) && (rx_data <= 64))
                            state <= P_DATA;
                        else begin
                            state <= P_H1;
                            frame_error <= 1'b1;
                        end
                    end

                    P_DATA: begin
                        case (data_count)
                            0: data0 <= rx_data;
                            1: data1 <= rx_data;
                            2: data2 <= rx_data;
                            3: data3 <= rx_data;
                            4: data4 <= rx_data;
                            5: data5 <= rx_data;
                            6: data6 <= rx_data;
                            7: data7 <= rx_data;
                            8: data8 <= rx_data;
                            9: data9 <= rx_data;
                            10: data10 <= rx_data;
                            11: data11 <= rx_data;
                            default: ;
                        endcase
                        sum <= sum + rx_data;
                        if (data_count == data_len - 1'b1)
                            state <= P_SUM;
                        else
                            data_count <= data_count + 1'b1;
                    end

                    P_SUM: begin
                        state <= P_H1;
                        if (sum == rx_data) begin
                            if(response_header) begin
                                if(data_len==1) case(frame_id)
                                  8'h03:if(data0<=3) begin gyro_range<=data0[1:0];gyro_known<=1;end
                                  8'h04:if(data0<=3) begin acc_range<=data0[1:0];acc_known<=1;end
                                  8'h0B:if(data0<=1) begin algorithm_mag<=data0[0];algorithm_known<=1;end
                                  8'h0C:if(data0<=1) begin mounting_vertical<=data0[0];mounting_known<=1;end
                                        else frame_error<=1;
                                  default:;
                                endcase
                            end else case (frame_id)
                                8'h01: begin
                                    // MS901M Z-Y-X Pitch is defined only in
                                    // [-90,+90] degrees: +/-16384 raw counts.
                                    // A checksum validates bytes, not attitude
                                    // semantics; keep the previous whole tuple
                                    // when this documented range is violated.
                                    if (data_len == 8'd6 &&
                                        $signed({data3,data2})>=-16'sd16384 &&
                                        $signed({data3,data2})<=16'sd16384) begin
                                        roll_raw  <= $signed({data1, data0});
                                        pitch_raw <= $signed({data3, data2});
                                        yaw_raw   <= $signed({data5, data4});
                                        attitude_valid <= 1'b1;
                                    end else frame_error <= 1'b1;
                                end
                                8'h02:begin
                                    if(data_len==8 && {data7,data6,data5,data4,data3,data2,data1,data0}!=64'd0)begin
                                        q0_raw<={data1,data0};q1_raw<={data3,data2};q2_raw<={data5,data4};q3_raw<={data7,data6};quaternion_valid<=1;
                                    end else frame_error<=1;
                                end
                                8'h04:begin
                                    if(data_len==8)begin
                                        mx_raw<={data1,data0};my_raw<={data3,data2};mz_raw<={data5,data4};mag_temperature_raw<={data7,data6};magnetic_valid<=1;
                                    end else frame_error<=1;
                                end
                                8'h03: begin
                                    if (data_len == 8'd12) begin
                                        ax_raw <= $signed({data1, data0});
                                        ay_raw <= $signed({data3, data2});
                                        az_raw <= $signed({data5, data4});
                                        gx_raw <= $signed({data7, data6});
                                        gy_raw <= $signed({data9, data8});
                                        gz_raw <= $signed({data11, data10});
                                        motion_valid <= 1'b1;
                                    end else frame_error <= 1'b1;
                                end
                                8'h05: begin
                                    if (data_len == 8'd10) begin
                                        pressure_raw <= $signed({data3, data2, data1, data0});
                                        altitude_cm <= $signed({data7, data6, data5, data4});
                                        temperature_raw <= $signed({data9, data8});
                                        altitude_valid <= 1'b1;
                                    end else frame_error <= 1'b1;
                                end
                                default: ;
                            endcase
                        end else begin
                            frame_error <= 1'b1;
                        end
                    end

                    default: state <= P_H1;
                endcase
            end
        end
    end
endmodule
