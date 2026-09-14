/*
 * Etapa 6: RTL - TinyOpt-4 (Verilog-2001)
 * Motor adaptativo LMS de 4 coeficientes en punto fijo (Q1.7).
 */

`default_nettype none

module tt_um_tinyopt4 (
    input  wire [7:0] ui_in,    // Bus de datos principal (x, d, cfg)
    output wire [7:0] uo_out,   // Bus de salida multiplexado (y_hat, e, w_i)
    input  wire [7:0] uio_in,   // Pines bidireccionales (Entrada)
    output wire [7:0] uio_out,  // Pines bidireccionales (Salida)
    output wire [7:0] uio_oe,   // Direccionalidad de uio (1 = output, 0 = input)
    input  wire       ena,      // Habilitador del diseño (Tiny Tapeout wrapper)
    input  wire       clk,      // Reloj principal (Objetivo 10-20 MHz)
    input  wire       rst_n     // Reset asíncrono, activo en bajo
);

    // ==========================================
    // 1. DEFINICIÓN DE ESTADOS (FSM)
    // ==========================================
    localparam [3:0] 
        S_IDLE = 4'd0,
        S_MAC0 = 4'd1,
        S_MAC1 = 4'd2,
        S_MAC2 = 4'd3,
        S_MAC3 = 4'd4,
        S_ERR  = 4'd5,
        S_UPD0 = 4'd6,
        S_UPD1 = 4'd7,
        S_UPD2 = 4'd8,
        S_UPD3 = 4'd9,
        S_DONE = 4'd10;

    reg [3:0] state;

    // ==========================================
    // 2. REGISTROS INTERNOS Y DATAPATH
    // ==========================================
    reg signed [7:0]  x0, x1, x2, x3;   // Línea de retardo
    reg signed [7:0]  w0, w1, w2, w3;   // Pesos (Q1.7)
    reg signed [7:0]  d;                // Señal deseada
    reg signed [17:0] acc;              // Acumulador MAC
    reg signed [7:0]  y_hat;            // Predicción saturada
    reg signed [7:0]  e;                // Error saturado
    reg [2:0]         mu_s;             // Factor de desplazamiento (Tasa de aprendizaje)
    
    // Banderas de estado
    reg flag_overflow;
    reg flag_weight_sat;

    // ==========================================
    // 3. MAPEO DE PINES UIO
    // ==========================================
    // Entradas [3:0]:
    wire start    = uio_in[0];
    wire data_sel = uio_in[1];
    wire cfg_sel  = uio_in[2];
    
    // Configuración bidireccional (0: input, 1: output)
    assign uio_oe = 8'b1111_0000; 

    // ==========================================
    // 4. LÓGICA COMBINACIONAL (RECURSOS COMPARTIDOS)
    // ==========================================
    
    // --- 4.1. Multiplicador Compartido (8x8 -> 16 bit firmado) ---
    reg signed [7:0] mult_a;
    reg signed [7:0] mult_b;
    wire signed [15:0] mult_out = mult_a * mult_b;

    always @* begin
        // Multiplexor de entradas al multiplicador según el estado
        case (state)
            S_MAC0: begin mult_a = w0; mult_b = x0; end
            S_MAC1: begin mult_a = w1; mult_b = x1; end
            S_MAC2: begin mult_a = w2; mult_b = x2; end
            S_MAC3: begin mult_a = w3; mult_b = x3; end
            S_UPD0: begin mult_a = e;  mult_b = x0; end
            S_UPD1: begin mult_a = e;  mult_b = x1; end
            S_UPD2: begin mult_a = e;  mult_b = x2; end
            S_UPD3: begin mult_a = e;  mult_b = x3; end
            default:begin mult_a = 8'sd0; mult_b = 8'sd0; end
        endcase
    end

    // --- 4.2. Saturación y Truncamiento de y_hat ---
    wire signed [17:0] acc_shifted = acc >>> 7; // Vuelta a Q1.7
    wire acc_ovf = (acc_shifted > 18'sd127) || (acc_shifted < -18'sd128);
    wire signed [7:0] y_hat_nxt = (acc_shifted > 18'sd127)  ?  8'sd127 :
                                  (acc_shifted < -18'sd128) ? -8'sd128 : 
                                  acc_shifted[7:0];

    // --- 4.3. Saturación de Error ---
    wire signed [8:0] err_calc = d - y_hat;
    wire err_ovf = (err_calc > 9'sd127) || (err_calc < -9'sd128);
    wire signed [7:0] err_nxt = (err_calc > 9'sd127)  ?  8'sd127 :
                                (err_calc < -9'sd128) ? -8'sd128 : 
                                err_calc[7:0];

    // --- 4.4. Gradiente y Saturación de Pesos ---
    // Desplazamiento dinámico seguro para el gradiente
    wire [3:0] shift_amt = 4'd7 + mu_s; 
    wire signed [15:0] grad_shifted = mult_out >>> shift_amt;

    // Selector del peso actual para el acumulador de actualización
    wire signed [15:0] w_current = (state == S_UPD0) ? {{8{w0[7]}}, w0} :
                                   (state == S_UPD1) ? {{8{w1[7]}}, w1} :
                                   (state == S_UPD2) ? {{8{w2[7]}}, w2} :
                                   (state == S_UPD3) ? {{8{w3[7]}}, w3} : 16'sd0;

    wire signed [15:0] w_target = w_current + grad_shifted;
    wire w_sat_flag = (w_target > 16'sd127) || (w_target < -16'sd128);
    
    wire signed [7:0] w_nxt = (w_target > 16'sd127)  ?  8'sd127 :
                              (w_target < -16'sd128) ? -8'sd128 : 
                              w_target[7:0];

    // ==========================================
    // 5. FSM SECUENCIAL Y ACTUALIZACIÓN DE REGISTROS
    // ==========================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
            x0 <= 8'sd0; x1 <= 8'sd0; x2 <= 8'sd0; x3 <= 8'sd0;
            w0 <= 8'sd0; w1 <= 8'sd0; w2 <= 8'sd0; w3 <= 8'sd0;
            d <= 8'sd0; acc <= 18'sd0; y_hat <= 8'sd0; e <= 8'sd0;
            mu_s <= 3'd3; // Valor por defecto
            flag_overflow <= 1'b0;
            flag_weight_sat <= 1'b0;
        end else begin
            case (state)
                S_IDLE: begin
                    flag_overflow <= 1'b0;
                    flag_weight_sat <= 1'b0;
                    if (start) begin
                        if (cfg_sel) begin
                            mu_s <= ui_in[2:0];
                        end else if (data_sel == 1'b0) begin
                            // Desplazar línea de retardo
                            x3 <= x2; x2 <= x1; x1 <= x0;
                            x0 <= ui_in;
                        end else if (data_sel == 1'b1) begin
                            // Iniciar iteración matemática
                            d <= ui_in;
                            state <= S_MAC0;
                        end
                    end
                end
                
                S_MAC0: begin acc <= mult_out; state <= S_MAC1; end
                S_MAC1: begin acc <= acc + mult_out; state <= S_MAC2; end
                S_MAC2: begin acc <= acc + mult_out; state <= S_MAC3; end
                S_MAC3: begin acc <= acc + mult_out; state <= S_ERR; end
                
                S_ERR: begin
                    y_hat <= y_hat_nxt;
                    e <= err_nxt;
                    if (acc_ovf || err_ovf) flag_overflow <= 1'b1;
                    state <= S_UPD0;
                end
                
                S_UPD0: begin 
                    w0 <= w_nxt; 
                    if (w_sat_flag) flag_weight_sat <= 1'b1;
                    state <= S_UPD1; 
                end
                S_UPD1: begin 
                    w1 <= w_nxt; 
                    if (w_sat_flag) flag_weight_sat <= 1'b1;
                    state <= S_UPD2; 
                end
                S_UPD2: begin 
                    w2 <= w_nxt; 
                    if (w_sat_flag) flag_weight_sat <= 1'b1;
                    state <= S_UPD3; 
                end
                S_UPD3: begin 
                    w3 <= w_nxt; 
                    if (w_sat_flag) flag_weight_sat <= 1'b1;
                    state <= S_DONE; 
                end
                
                S_DONE: begin state <= S_IDLE; end
                default: state <= S_IDLE;
            endcase
        end
    end

    // ==========================================
    // 6. ASIGNACIÓN DE SALIDAS (MULTIPLEXOR DE LECTURA)
    // ==========================================
    reg [7:0] uo_out_reg;
    always @* begin
        // Lectura combinacional asíncrona visible cuando IDLE
        case (ui_in[2:0])
            3'b000: uo_out_reg = y_hat;
            3'b001: uo_out_reg = e;
            3'b010: uo_out_reg = w0;
            3'b011: uo_out_reg = w1;
            3'b100: uo_out_reg = w2;
            3'b101: uo_out_reg = w3;
            default: uo_out_reg = 8'd0;
        endcase
    end

    assign uo_out = uo_out_reg;

    // Salidas de bandera hacia el wrapper UIO
    wire busy = (state != S_IDLE && state != S_DONE);
    wire done = (state == S_DONE);
    
    // Mapeo: [7]=WeightSat, [6]=Overflow, [5]=Done, [4]=Busy
    assign uio_out = {flag_weight_sat, flag_overflow, done, busy, 4'b0000};

endmodule