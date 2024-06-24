                                            ;******************************************************************************
 ;***  Marvin Castro Castro - Microprocesadores - EIE UCR
 ;***           PROYECTO FINAL: RADAR 623
 ;******************************************************************************
#include registers.inc
 ;******************************************************************************
 ;                 RELOCALIZACION DE VECTOR DE INTERRUPCION
 ;******************************************************************************
                                Org $3E64       ; vector Output Compare
                                dw Maquina_Tiempos

;******************************************************************************
;                   DECLARACION DE LOS VALORES DEL PROGRAMA
;******************************************************************************

; ------------------------------ TAREA TECLADO ---------------------------------
tSuprRebTCL               EQU 20

; ---------------------------- TAREA PANTALLA_MUX ------------------------------
tTimerDigito            EQU 2
MaxCountTicks           EQU 100


; -------------------------------- TAREA LCD  ----------------------------------
tTimer260uS:              EQU 13
tTimer40uS:               EQU 2
tTimer2mS:                EQU 2
EOB                     EQU $FF
Clear_LCD               EQU $01
ADD_L1                  EQU $80
ADD_L2                  EQU $C0

; ------------------------- TAREAS LEER_PB1, LEER_PB0 --------------------------
PortPB             EQU PTIH
tSupRebPB0         EQU 10
tSupRebPB1         EQU 10
tShortP0           EQU 25
tShortP1           EQU 25
tLongP0            EQU 3
tLongP1            EQU 3
MaskPB1            EQU $04
MaskPB0            EQU $01

; ---------------------------- TAREA CONFIGURAR --------------------------------

;LDConfig         EQU
;LimMax           EQU
;LimMin           EQU

; ---------------------------- TAREA INACTIVO ----------------------------------
;LDInac           EQU
;tTimerVLim       EQU

; --------------------------- TAREA EnServicio ---------------------------------

;LDEnServ          EQU
;tTimerVel         EQU
;tTimerError       EQU
;VelocMin          EQU
;VelocMax          EQU

; ------------------------------ TAREA Brillo ----------------------------------
tTimerBrillo       EQU 4
MaskSCF            EQU $80

; ------------------------------ TAREA LeerDS ----------------------------------
tTimerRebDS       EQU 10

; --------------------------- TAREA DesplazarLeds ------------------------------
;tTimerDplzLeds

; ------------------------------- Banderas -------------------------------------
; ----- Banderas_1
ShortP0           EQU $01
LongP0            EQU $02
ShortP1           EQU $04
LongP1            EQU $08
Array_OK          EQU $10
; ----- Banderas_2
RS                EQU $01
LCD_OK            EQU $02
FinSendLCD        EQU $04
SecondLine        EQU $08
Alarma            EQU $10
DsplzIzquierda    EQU $20
; -------------------------------- GENERALES -----------------------------------
tTimerLDTst       EQU 5     ;Tiempo de parpadeo de LED testigo en segundos


;--- Aqui se colocan los valores de carga para los timers baseT  ----

tTimer1mS:        EQU 50     ;Base de tiempo de 1 mS (1 ms x 1 / 20us)
tTimer10mS:       EQU 500    ;Base de tiempo de 10 mS (1 mS x 10 / 20us)
tTimer100mS:      EQU 5000    ;Base de tiempo de 100 mS (10 mS x 100 / 20us)
tTimer1S:         EQU 50000    ;Base de tiempo de 1 segundo (100 mS x 10 / 20us)

;******************************************************************************
;                   DECLARACION DE LAS ESTRUCTURAS DE DATOS
;******************************************************************************



; ==============================================================================
;                            ESTRUCTURAS DE DATOS
                                Org $1000


; ----------------------------- TAREA_TECLADO ----------------------------------
MAX_TCL                 db 5
Tecla                   db $FF
Tecla_IN                db $FF
Cont_TCL                db $00
Patron                  db $00
Est_Pres_TCL            ds 2
                                org $1010
Num_Array               db $FF,$FF,$FF,$FF,$FF

; ---------------------------- TAREA PANTALLA_MUX ------------------------------
                                org $1020
EstPres_PantallaMUX     ds 2
Dsp1                    ds 1
Dsp2                    ds 1
Dsp3                    ds 1
Dsp4                    ds 1
LEDS                    ds 1
Cont_Dig                ds 1
Brillo                  db 100
BIN1                    ds 1
BIN2                    ds 1

; --------------------------- Subrutina CONVERSION  ----------------------------
BCD                     ds 1
Cont_BCD                ds 1
BCD1                    ds 1
BCD2                    ds 1 ;$102e

; -------------------------------- TAREA LCD  ----------------------------------
IniDsp                  dB $28,$28,$06,$0C,$FF ;1033
Punt_LCD                ds 2
CharLCD                 ds 1
Msg_L1                  ds 2
Msg_L2                  ds 2
EstPres_SendLCD         ds 2
EstPres_TareaLCD        ds 2  ;$103d

; ------------------------- TAREAS LEER_PB1, LEER_PB0 --------------------------
Est_Pres_LeerPB0   ds 2
Est_Pres_LeerPB1   ds 2

; ---------------------------- TAREA CONFIGURAR --------------------------------
Est_Pres_TConfig  ds 2
ValorLIM          ds 1
Vel_LIM           dB 65

; ---------------------------- TAREA INACTIVO ---------------------------------
Est_Pres_TInac    ds 2

; ----------------------------- TAREA EnServicio -------------------------------
Est_Pres_TServ    ds 2
Vel_Calc          ds 1
DeltaT            ds 1

; ------------------------------ TAREA Brillo ----------------------------------
Est_Pres_TBrillo   ds 2

; ------------------------------ TAREA LeerDS ----------------------------------
Est_Pres_LeerDS    ds 2
Temp_DS            ds 1
Valor_DS           ds 1

; --------------------------- TAREA DesplazarLeds ------------------------------
Est_Pres_DsplzLeds ds 2
DplzLeds           ds 1

; -------------------------------- BANDERAS ------------------------------------
                                org $1070
Banderas_1        ds 1
Banderas_2        ds 1
Banderas          ds 1

; -------------------------------- GENERALES -----------------------------------
LED_Testigo       ds 1

; --------------------------------- TABLAS -------------------------------------
                                    org $1100
;-------------------------------------------------------------------------------
;                              TABLA DE SEGMENT

Segment         dB $3F,$06,$5B,$4F,$66,$6D,$7D,$07,$7F,$6F
;-------------------------------------------------------------------------------
;                              TABLA DE TECLAS

Teclas            dB $01,$02,$03,$04,$05,$06,$07,$08,$09,$0B,$00,$0E

;===============================================================================
;                                  MENSAJES
;===============================================================================
                                   org $1200
MSG1            FCC "ING ELECTRICA"
                db EOB
MSG2            FCC "UCR 2024"
                db EOB
MSG3            FCC "uPROCESADORES"
                db EOB
MSG4            FCC "IE0623"
                db EOB
MSG_CONFIG1     FCC " MODO CONFIGURAR"
                db EOB
MSG_CONFIG2     FCC "         VEL_LIM"
                db EOB


;===============================================================================
;                              TABLA DE TIMERS
;===============================================================================
                                Org $1500
Tabla_Timers_BaseT:

Timer1mS        ds 2       ;1040-1041
Timer10mS:      ds 2       ;1042-1043
Timer100mS:     ds 2       ;1044-1045
Timer1S:        ds 2       ;1045-1046
Timer260uS:     ds 2
Timer40uS:      ds 2

Counter_Ticks   ds 2       ;1047-1048

Fin_BaseT       dW $FFFF   ;1049-104A

Tabla_Timers_Base1mS

Timer_Digito    ds 1      ;104B
Timer_RebPB0    ds 1      ;104C
Timer_RebPB1    ds 1      ;104C
Timer_RebTCL    ds 1      ;104D
Timer2mS        ds 1
TimerRebDS      ds 1


;Timer1_Base1:   ds 1       ;Ejemplos de timers de aplicacion con BaseT
;Timer2_Base1:   ds 1

Fin_Base1mS:    dB $FF

Tabla_Timers_Base10mS

Timer_SHP0       ds 1
Timer_SHP1       ds 1
;Timer1_Base10:  ds 1       ;Ejemplos de timers de aplicacion con base 10 mS
;Timer2_Base10:  ds 1

Fin_Base10ms    dB $FF

Tabla_Timers_Base100mS

Timer1_100mS    ds 1
Timer_LED_Testigo ds 1   ;Timer para parpadeo de led testigo
TimerBrillo       ds 1
;Timer1_Base100  ds 1       ;Ejemplos de timers de aplicacpon con base 100 mS
;Timer2_Base100  ds 1

Fin_Base100mS   dB $FF

Tabla_Timers_Base1S


Timer_LP0          ds 1
Timer_LP1          ds 1


;Timer1_Base1S:    ds 1   ;Ejemplos de timers de aplicacion con base 1 seg.
;Timer2_Base1S:    ds 1

Fin_Base1S        dB $FF

;Tabla_Timers_Base260uS

;Timer1_260uS    ds 1
;Timer1_Base100  ds 1
;Timer2_Base100  ds 1

;Fin_Base260uS   dB $FF

;Tabla_Timers_Base40uS

;Timer1_40uS    ds 1
;Timer1_Base100  ds 1
;Timer2_Base100  ds 1

;Fin_Base40uS   dB $FF

;===============================================================================
;                              CONFIGURACION DE HARDWARE
;===============================================================================
                              Org $2000
; ------------------------ Bloque de configuración de LEDS ---------------------
        Bset DDRB,$FF
        Bset DDRJ,$02
        BClr PTJ,$02

; --------------Bloque de configuración de Disp. 7 Segmentos y Led RGB----------
        Movb #$7F,DDRP    ;Habilito (pongo como salidas) PP6-PP4 (RGB LED)
                          ; y PP3-PP0 (digitos de 7 segmentos)

; ---------------- Bloque de configuración de puerto de botones ----------------
        movb #$00,DDRH

; ---------------- Bloque de configuración de convertidor ATD ------------------
        movb #$20,ATD0CTL3
        movb #$C8,ATD0CTL4
        movb #$07,ATD0CTL5
; ---------------- Bloque de configuración Output Compare Canal 5 ------------
        Movb #$90,TSCR1
        Movb #$00,TSCR2
        Movb #$20,TIOS
        Movb #$20,TIE
        ;bset PTP,$40

        ldd TCNT
        addd #480
        std TC5
; ---------------- Bloque de configuración Teclado Matricial -------------------
        movb #$F0,DDRA     ; pongo como salidas los 4 msb
        movb #$01,PUCR     ; activo pull ups para puerto A
        clr Cont_TCL
;===============================================================================
;                           PROGRAMA PRINCIPAL
;===============================================================================
; ------------------------- Inicialización de timers ---------------------------
	Movw #tTimer1mS,Timer1mS
        Movw #tTimer10mS,Timer10mS         ;Inicia los timers de bases de tiempo
        Movw #tTimer100mS,Timer100mS
        Movw #tTimer1S,Timer1S

        movb #tTimerDigito,Timer_Digito
        Movb #tTimerLDTst,Timer_LED_Testigo  ;inicia timer parpadeo led testigo
        movb #1,Cont_Dig



        Lds #$3BFF
        Cli
        Clr Banderas_1
        Clr Banderas_2
        Clr LEDS
; ------------------- Inicialización de punteros de próximo estado -------------
        movw #TareaLeerDS_Est1,Est_Pres_LeerDS
        ;movw #TareaInactivo_Est1,Est_Pres_TInac
        movw #TareaConfig_Est1,Est_Pres_TConfig
        movw #LeerPB0_Est1,Est_Pres_LeerPB0
        movw #LeerPB1_Est1,Est_Pres_LeerPB1
        movw #TareaTCL_Est1,Est_Pres_TCL
        movw #TareaMuxPantalla_Est1,EstPres_PantallaMUX
        movw #TareaSendLCD_Est1,EstPres_SendLCD
        movw #TareaLCD_Est1,EstPres_TareaLCD
        movw #TareaBrillo_Est1,Est_Pres_TBrillo

        jsr Rutina_InitLCD
        movw #MSG1,Msg_L1
        movw #MSG2,Msg_L2
        bclr Banderas_2,LCD_OK
Despachador_Tareas
; ----- Mensajes LCD ------ ;
        brset Banderas_2,LCD_OK,skipLCD
        jsr Tarea_LCD
skipLCD
        Jsr Tarea_Led_Testigo
; ----------------- Aqui se colocan todas las tareas del programa de aplicacion
        jsr Tarea_Teclado
        Jsr Tarea_LeerPB0
        Jsr Tarea_LeerPB1
        jsr Tarea_LeerDS
        jsr ControlModo
        ;jsr Tarea_Borra_TCL
        ;Jsr Tarea_Led_PB
        ;jsr Tarea_Conversion
        jsr Tarea_Mux_Pantalla
        jsr Tarea_Brillo
        ;movb #$33,CharLCD
               ;jsr Tarea_SendLCD

        Bra Despachador_Tareas
;******************************************************************************
;                               TAREA MODO CONFIGURAR
;******************************************************************************
Tarea_Configurar
                ldx Est_Pres_TConfig
                jsr 0,x
       rts

;========================== CONFIGURAR ESTADO 1 ================================
TareaConfig_Est1

                movw #MSG_CONFIG2,Msg_L2
                movw #MSG_CONFIG1,Msg_L1
                bclr Banderas_2,LCD_OK

                ;brset Banderas_2,LCD_OK,return_config_est1



                ldaa Vel_LIM
                jsr Bin_BCD_MuxP
                movb BCD,BCD2
                bset DDRP,$03
                bset PTP,$03
                jsr BCD_7Seg
                jsr Borrar_NumArray

                movw #TareaConfig_Est2,Est_Pres_TConfig
return_config_est1
		rts
;========================== CONFIGURAR ESTADO 2 ================================
TareaConfig_Est2
                brclr Banderas_1,Array_OK,retConfig_est2
                jsr BCD_BIN
                cmpa #65
                blo est2_conf_borrartcl
                cmpa #90
                bhi est2_conf_borrartcl
                staa ValorLIM
                jsr Bin_BCD_MuxP
                movb BCD,BCD2
                bset DDRP,$03
                bset PTP,$03
                jsr BCD_7Seg
                movb ValorLIM,Vel_LIM
                
                
                
est2_conf_borrartcl
                jsr Borrar_NumArray
retConfig_est2  rts

;******************************************************************************
;                               Subrutina Control Modo
;******************************************************************************
; Se encarga de seleccionar el modo de operación según los valores de los DIP
; Switches 7 y 6.

ControlModo
                brset Valor_DS,$80,Dip7on
                brset Valor_DS,$40,Dip6on
                ; jsr Tarea_ModoInactivo
                bset LEDS,$01
                bclr LEDS,$02
                bclr LEDS,$04
                bra retControlModo
Dip6on          ; 0 1
                jsr Tarea_Configurar
                bclr LEDS,$01
                bset LEDS,$02
                bclr LEDS,$04
                bra  retControlModo

Dip7on          brset Valor_DS,$40,Dip7_6on
                bra retControlModo

Dip7_6on        ; 1 1
		; jsr Tarea_EnServicio
		bclr LEDS,$01
        	bclr LEDS,$02
        	bset LEDS,$04

retControlModo  rts
;******************************************************************************
;                               TAREA LeerDS
;******************************************************************************
Tarea_LeerDS
                ldx Est_Pres_LeerDS
                jsr 0,x
FinLeerDS       rts

;============================ LeerDS ESTADO 1 =================================
TareaLeerDS_Est1
                ldaa PTIH
                cmpa #0
                bls retLeerDS_est1
                movb PTIH,Temp_DS
                movw #TareaLeerDS_Est2,Est_Pres_LeerDS
                movb #tTimerRebDS,TimerRebDS
retLeerDS_est1  rts

;============================ LeerDS ESTADO 2 =================================
TareaLeerDS_Est2
                tst TimerRebDS
                bne retLeerDS_est2
                ldab PTIH
                cmpb Temp_DS
                bne  noise_LeerDS
                stab Valor_DS
                movw #TareaLeerDS_Est1,Est_Pres_LeerDS
noise_LeerDS
                movw #TareaLeerDS_Est1,Est_Pres_LeerDS
                bra retLeerDS_est2

retLeerDS_est2  rts
;******************************************************************************
;                               TAREA BRILLO
;******************************************************************************
Tarea_Brillo
                ldx Est_Pres_TBrillo
                jsr 0,x
FinTareaBrillo  rts

;============================ BRILLO ESTADO 1 =================================
TareaBrillo_Est1
                ; 2 conversiones de canal 7
                ; 4 periodos de reloj para muestreo
                ; 8 bits unsigned
                ; frec operacion 650 khz
                movb #tTimerBrillo,TimerBrillo
                movw #TareaBrillo_Est2,Est_Pres_TBrillo
                rts
;============================ BRILLO ESTADO 2 =================================
TareaBrillo_Est2
                tst TimerBrillo
                bne ret_brillo_est2
                movb #$80,ATD0CTL2
                movw #TareaBrillo_Est3,Est_Pres_TBrillo

ret_brillo_est2
                rts
;============================ BRILLO ESTADO 3 =================================
TareaBrillo_Est3
                brclr ATD0STAT0,MaskSCF,ret_brillo_est3
                ldd ADR00H
                addd ADR00H
                lsrd
                nop
                nop
                bset ATD0STAT0,MaskSCF
                movw #TareaBrillo_Est1,Est_Pres_TBrillo
                movb #$00,ATD0CTL2
                movb #$07,ATD0CTL5
                ;stab BCD2
                ;staa BCD1
                ; valor guardado en A
                ; transformacion: brillo =  ((acumA*100)/255))
                tab
                clra
                pshy
                ldy #100
                emul
                ;xgdy
                puly
                pshx
                ldx #255
                idiv
                xgdx
                pulx
                stab Brillo
                ;stab BIN1
                ;nop


ret_brillo_est3 rts
;******************************************************************************
;                               TAREA CONVERSION
;******************************************************************************
Tarea_Conversion
                        ldaa BIN1
                        jsr Bin_BCD_MuxP
                        movb BCD,BCD1



                        ldaa BIN2
                        jsr Bin_BCD_MuxP
                        movb BCD,BCD2



                        jsr BCD_7Seg

                        rts

;******************************************************************************
;                               Rutina InitLCD
;******************************************************************************
Rutina_InitLCD
                        movw #tTimer260uS,Timer260uS
                        movw #tTimer40uS,Timer40uS
                        movb #tTimer2mS,Timer2mS
                        movb #$FF,DDRK
                        clr Punt_LCD
                        bclr Banderas_2,RS   ; borro bandera RS  ;;;;;;;;;;;;;;;;;;;;;;;;;
                        bclr Banderas_2,SecondLine
                        bset Banderas_2,LCD_OK

                        ldy #IniDsp
loop_within_InitLCD
                        ldaa 1,y+
                        staa CharLCD

                        cmpa #EOB
                        bne not_eob

                        ; si es EOB
                        movb #Clear_LCD,CharLCD
                        nop
loop_for_clear          jsr Tarea_SendLCD
                        brset Banderas_2,FinSendLCD,endedClear
                        bra loop_for_clear

endedClear              movb #tTimer2mS,Timer2mS
wait_for_clear          tst Timer2mS
                        bne wait_for_clear
                        ;bclr Banderas_2,RS      ;;;;;;;;;;;;;;;;;;;;;;;;;;;
                        bra exit_InitLCD

not_eob                 jsr Tarea_SendLCD

                        brset Banderas_2,FinSendLCD,clearFinSendLCD
                        bra not_eob
clearFinSendLCD
                        bclr Banderas_2,FinSendLCD
                        bra loop_within_InitLCD


exit_InitLCD            rts

;******************************************************************************
;                               TAREA LCD
;******************************************************************************
Tarea_LCD
                ldx EstPres_TareaLCD
                jsr 0,x
FinTareaLCD     rts
;============================ LCD ESTADO 1 =================================
TareaLCD_Est1
                        bclr Banderas_2,FinSendLCD
                        bclr Banderas_2,RS

                        brset Banderas_2,SecondLine,sendSecondLine
                        movb #ADD_L1,CharLCD
                        movw Msg_L1,Punt_LCD
                        bra goto_SendLCD
sendSecondLine
                        movb #ADD_L2,CharLCD
                        movw Msg_L2,Punt_LCD
goto_SendLCD
                        jsr Tarea_SendLCD
                        movw #TareaLCD_Est2,EstPres_TareaLCD
                        rts
;============================ LCD ESTADO 2 =================================
TareaLCD_Est2
                        brset Banderas_2,FinSendLCD,ended_item
sendlcd_then_return     jsr Tarea_SendLCD
                        bra retLCD_est2
ended_item
                        bclr Banderas_2,FinSendLCD
                        bset Banderas_2,RS   ; RS on - enviar datos

                        ldy Punt_LCD ; y = 1200
                        ldaa 0,y
                        staa CharLCD

                        ;movb Punt_LCD,CharLCD
                        iny
                        sty Punt_LCD

                        ;ldaa CharLCD
                        cmpa #EOB
                        bne sendlcd_then_return
                        brset Banderas_2,SecondLine,LCD2_SecondLine
                        bset Banderas_2,SecondLine
                        bra retLCD_est2_to_est1

LCD2_SecondLine
                        bclr Banderas_2,SecondLine
                        bset Banderas_2,LCD_OK

retLCD_est2_to_est1     movw #TareaLCD_Est1,EstPres_TareaLCD
retLCD_est2             rts




;******************************************************************************
;                               TAREA SendLCD
;******************************************************************************
Tarea_SendLCD
                ldx EstPres_SendLCD
                jsr 0,x
FinTareaSLCD     rts
;============================ SendLCD ESTADO 1 =================================
TareaSendLCD_Est1
                        ldaa CharLCD
                        anda #$F0
                        lsra
                        lsra
                        staa PORTK

                        brclr Banderas_2,RS,isACommand
                        bset PORTK,$01  ; RS on
                        bra ret_SendLCD_est1

isACommand              bclr PORTK,$01  ; RS off

ret_SendLCD_est1
                        bset PORTK,$02   ; EN on
                        movw #tTimer260uS,Timer260uS
                        movw #TareaSendLCD_Est2,EstPres_SendLCD
                        rts
;============================ SendLCD ESTADO 2 ===============================
TareaSendLCD_Est2
                        ldx Timer260uS
                        bne retret_SendLCD_est2
                        bclr PORTK,$02   ; EN off
                        ldaa CharLCD
                        anda #$0F
                        lsla
                        lsla
                        staa PORTK

                        brclr Banderas_2,RS,isACommand2
                        bset PORTK,$01
                        bra ret_SendLCD_est2
isACommand2
                        bclr PORTK,$01

ret_SendLCD_est2
                        bset PORTK,$02  ; EN on
                        movw #tTimer260uS,Timer260uS
                        movw #TareaSendLCD_Est3,EstPres_SendLCD
retret_SendLCD_est2     rts
;============================ SendLCD ESTADO 3 ===============================
TareaSendLCD_Est3
                        ldx Timer260uS
                        bne ret_SendLCD_est3
                        bclr PORTK,$02
                        movw #tTimer40uS,Timer40uS
                        movw #TareaSendLCD_Est4,EstPres_SendLCD

ret_SendLCD_est3        rts

;============================ SendLCD ESTADO 4 ================================
TareaSendLCD_Est4
                        ldx Timer40uS
                        bne ret_SendLCD_est4
                        bset Banderas_2,FinSendLCD ; mascara para FinSendLCD
                        movw #TareaSendLCD_Est1,EstPres_SendLCD

ret_SendLCD_est4        rts







;******************************************************************************
;                               TAREA MUX_PANTALLA
;******************************************************************************
Tarea_Mux_Pantalla

                ldx EstPres_PantallaMUX
                jsr 0,x
FinTareaPMUX    rts

;=========================== MUX_PANTALLA ESTADO 1 =============================
TareaMuxPantalla_Est1
                        tst Timer_Digito
                        lbne ret_pmux_est1
                        movb #tTimerDigito,Timer_Digito
                        ldaa Cont_Dig
                        cmpa #1
                        beq goDig1
                        cmpa #2
                        beq goDig2
                        cmpa #3
                        beq goDig3
                        cmpa #4
                        beq goDig4

                        bclr PTJ,$02
;                        movb #$55,LEDS
                        movb LEDS,PORTB
                        movb #1,Cont_Dig

                        bset PTP,$0F
                        bra pmux_prox_est2




goDig1                  ;movb #$0E,PTP
                        bset PTP,$0E  ; ---------
                        bclr PTP,$01  ; ---------
                        ;bclr PTP,$01
                        movb Dsp1,PORTB
                        inc Cont_Dig
                        bra pmux_prox_est2
goDig2
                         ;movb #$0D,PTP
                        ;movb PTP,$0D
                        bset PTP,$0D  ; ---------  0D
                       ;bclr PTP,$B2  ; ---------   B2
                        bclr PTP,$02
                        movb Dsp2,PORTB
                        inc Cont_Dig
                        bra pmux_prox_est2
goDig3
                        ;movb #$0B,PTP
                        bset PTP,$0B  ; ---------
                        bclr PTP,$04  ; ---------
                        ;bclr PTP,$04
                        movb Dsp3,PORTB
                        inc Cont_Dig
                        bra pmux_prox_est2
goDig4
                         ;movb #$07,PTP
                        bset PTP,$07  ; ---------       ; PTP,$07
                        ;bclr PTP,$B8  ; ---------        ; PTP,$B8
                                bclr PTP,$08
                        movb Dsp4,PORTB
                        inc Cont_Dig



pmux_prox_est2
                        movw #TareaMuxPantalla_Est2,EstPres_PantallaMUX
                        movw #100,Counter_Ticks

ret_pmux_est1           rts

;=========================== MUX_PANTALLA ESTADO 2 =============================
TareaMuxPantalla_Est2
                        ldd #100
                        subd Counter_Ticks

                        cmpb Brillo
                        blo ret_pmux_est2

                        bset PTP,$0F
                        bset PTJ,$02
                        movb #$00,PORTB
                        ;bset PTP,$BF
                        movw #TareaMuxPantalla_Est1,EstPres_PantallaMUX

ret_pmux_est2           rts
;******************************************************************************
;                               TAREA LED TESTIGO
;******************************************************************************

Tarea_Led_Testigo
                Tst Timer_LED_Testigo
                Bne FinLedTest
                Movb #tTimerLDTst,Timer_LED_Testigo
                brset PTP,$10,led1_on ; led 1
                brset PTP,$20,led2_on


                bclr PTP,$40
                bset PTP,$10
                bra FinLedTest
led1_on
                bclr PTP,$10
                bset PTP,$20 ; 0 1 0 0 _0 0 0 0
                bra FinLedTest
led2_on
                bclr PTP,$20
                bset PTP,$40



                ;clra
FinLedTest
      Rts
;******************************************************************************
;                               Subrutina BCD_BIN
;******************************************************************************
BCD_BIN
                ldx #Num_Array
                ldaa 1,x+
                lsla
                lsla
                lsla
                lsla
                ldab 0,x
                aba
                
                psha
                anda #$F0
                lsra
                tab
                lsrb
                lsrb
                aba
                
                pulb
                andb #$0F
                aba
                rts
                

;******************************************************************************
;                               Subrutina BORRAR_NUMARRAY
;******************************************************************************
Borrar_NumArray
                ldx #Num_Array
                dex
                ldaa #$FF
                ldab MAX_TCL
                ;bclr Banderas_1,Array_OK

borrar_na_loop
                staa b,x
                decb
                bne borrar_na_loop
	   	rts


;******************************************************************************
;                               TAREA LED_PB
;******************************************************************************
Tarea_LED_PB
                brset Banderas_1,ShortP1,ON
                brset Banderas_1,LongP1,OFF
                bra FIN_Led
ON              BClr Banderas_1,LongP1
                Bset PORTB,$01
                bra FIN_Led
OFF             bclr Banderas_1,ShortP1
                bclr PORTB,$01

FIN_Led         rts
;******************************************************************************
;                               TAREA LEER_PB0
;******************************************************************************
Tarea_LeerPB0
                ldx Est_Pres_LeerPB0
                jsr 0,x
FinTareaPB0      rts

;============================= LEER_PB0 ESTADO 1 ================================
LeerPB0_Est1    brset PortPB,MaskPB0,ret_LeerPB0_Est1
                movb #tSupRebPB0,Timer_RebPB0
                movb #tShortP0,Timer_SHP0
                movb #tLongP0,Timer_LP0
                movw #LeerPB0_Est2,Est_Pres_LeerPB0

ret_LeerPB0_Est1  rts
;============================= LEER_PB0 ESTADO 2 ================================
LeerPB0_Est2
                tst Timer_RebPB0
                bne ret_LeerPB0_Est2
                brset PortPB,MaskPB0,est2_prox_est1_pb0
                movw #LeerPB0_Est3,Est_Pres_LeerPB0
                bra ret_LeerPB0_Est2

est2_prox_est1_pb0      movw #LeerPB0_Est1,Est_Pres_LeerPB0

ret_LeerPB0_Est2 rts
;============================= LEER_PB ESTADO 3 ================================
LeerPB0_Est3
                tst Timer_SHP0
                bne ret_LeerPB0_Est3
                brset PortPB,MaskPB0,est3_pb0_est1

                movw #LeerPB0_Est4,Est_Pres_LeerPB0
                bra ret_LeerPB0_Est3

est3_pb0_est1   bset Banderas_1,ShortP0
                movw #LeerPB0_Est1,Est_Pres_LeerPB0

ret_LeerPB0_Est3 rts

;============================= LEER_PB ESTADO 4 ================================
LeerPB0_Est4
                tst Timer_LP0
                bne tst_shortp0
                ; Si el PC llega aca, es que verifica LP
                brclr PortPB,MaskPB0,ret_LeerPB0_Est4
                nop
                bset  Banderas_1,LongP0

                nop
                bra leerpb0_prox_est1


tst_shortp0    brclr PortPB,MaskPB0,ret_LeerPB0_Est4
              bset  Banderas_1,ShortP0


leerpb0_prox_est1  movw #LeerPB0_Est1,Est_Pres_LeerPB0
ret_LeerPB0_Est4 rts


;******************************************************************************
;                               TAREA LEER_PB1
;******************************************************************************
Tarea_LeerPB1
                ldx Est_Pres_LeerPB1
                jsr 0,x
FinTareaPB1      rts

;============================= LEER_PB ESTADO 1 ================================
LeerPB1_Est1     brset PortPB,MaskPB1,ret_LeerPB1_Est1
                movb #tSupRebPB1,Timer_RebPB1
                movb #tShortP1,Timer_SHP1
                movb #tLongP1,Timer_LP1
                movw #LeerPB1_Est2,Est_Pres_LeerPB1

ret_LeerPB1_Est1  rts
;============================= LEER_PB ESTADO 2 ================================
LeerPB1_Est2
                tst Timer_RebPB1
                bne ret_LeerPB1_Est2
                brset PortPB,MaskPB1,est2_pb1_est1
                movw #LeerPB1_Est3,Est_Pres_LeerPB1
                bra ret_LeerPB1_Est2

est2_pb1_est1      movw #LeerPB1_Est1,Est_Pres_LeerPB1

ret_LeerPB1_Est2 rts
;============================= LEER_PB ESTADO 3 ================================
LeerPB1_Est3
                tst Timer_SHP1
                bne ret_LeerPB1_Est3
                brset PortPB,MaskPB1,est3_pb1_est1

                movw #LeerPB1_Est4,Est_Pres_LeerPB1
                bra ret_LeerPB1_Est3

est3_pb1_est1   bset Banderas_1,ShortP1
                ;bset Banderas,ShortP1
                movw #LeerPB1_Est1,Est_Pres_LeerPB1

ret_LeerPB1_Est3 rts

;============================= LEER_PB ESTADO 4 ================================
LeerPB1_Est4
                tst Timer_LP1
                bne tst_shortp1
                ; Si el PC llega aca, es que verifica LP
                brclr PortPB,MaskPB1,ret_LeerPB1_Est4
                nop
                bset  Banderas_1,LongP1
                ;bset Banderas,LongP
                nop
                bra est4_pb1_est1


tst_shortp1    brclr PortPB,MaskPB1,ret_LeerPB1_Est4
              bset  Banderas_1,ShortP1
              ;bset Banderas,ShortP

est4_pb1_est1  movw #LeerPB1_Est1,Est_Pres_LeerPB1
ret_LeerPB1_Est4 rts

;******************************************************************************
;                               TAREA TECLADO
;******************************************************************************
Tarea_Teclado
                        ldx Est_Pres_TCL
                        jsr 0,x
FinTareaTCL             rts
;========================== TAREA_TECLADO ESTADO 1 =============================
TareaTCL_Est1
                        ;movb #$FF,Tecla
                        ;movb #$F5,Tecla_IN
                        jsr Leer_Teclado
                        staa Tecla
                        cmpa #$FF
                        beq ret_Teclado
                        ;movb Tecla,Tecla_IN
                        Movb #tSuprRebTCL,Timer_RebTCL
                        movw #TareaTCL_Est2,Est_Pres_TCL
ret_Teclado             rts

;========================== TAREA_TECLADO ESTADO 2 =============================
TareaTCL_Est2           tst Timer_RebTCL
                        bne ret_Teclado_est2
                        jsr Leer_Teclado
                        staa Tecla_IN
                        ldaa Tecla
                        cmpa Tecla_IN
                        bne est2_Teclado_prox1
                        movw #TareaTCL_Est3,Est_Pres_TCL
                        bra ret_Teclado_est2
est2_Teclado_prox1      movw #TareaTCL_Est1,Est_Pres_TCL


ret_Teclado_est2        rts
;========================== TAREA_TECLADO ESTADO 3 =============================
TareaTCL_Est3
                        jsr Leer_Teclado
                        brset PORTA,$0F,Tecla_Liberada
                        bra ret_Teclado_est3

Tecla_Liberada          movw #TareaTCL_Est4,Est_Pres_TCL
ret_Teclado_est3        rts
;========================== TAREA_TECLADO ESTADO 4 =============================
TareaTCL_Est4
                        ldx #Num_Array
                        ldaa MAX_TCL
                        deca
                        ldab a,x
                        cmpb #$FF
                        bne LongMaximaAlcanzada
                        ; no se ha llenado el array
                        clra
                        ldab a,x
                        cmpb #$FF

                        bne NoEsPrimeraTecla
                        ; si es la primera tecla
                        ldab Tecla
                        cmpb #$0B
                        beq exit_tcl_est4
                        cmpb #$0E
                        beq exit_tcl_est4
                        ldaa Cont_TCL
                        ldab Tecla
                        stab a,x
                        inc Cont_TCL
                        bra exit_tcl_est4

NoEsPrimeraTecla
                        ldab Tecla
                        cmpb #$0B
                        beq NoPrimera_EsBorrar
                        cmpb #$0E
                        beq NoPrimera_EsEnter
                        ldaa Cont_TCL
                        ldab Tecla
                        stab a,x
                        inc Cont_TCL
                        bra exit_tcl_est4
NoPrimera_EsEnter
                        clr Cont_TCL
                        bset Banderas_1,Array_OK
                        bra exit_tcl_est4
NoPrimera_EsBorrar
                        ldaa Cont_TCL
                        beq exit_tcl_est4
                        ldab #$FF
                        stab a,x
                        dec Cont_TCL
                        bra exit_tcl_est4
LongMaximaAlcanzada
                        ldab Tecla
                        cmpb #$0B
                        beq UltimaTecla_EsBorrar
                        cmpb #$0E
                        beq UltimaTecla_EsEnter
                        bra exit_tcl_est4

UltimaTecla_EsBorrar
                        ldab #$FF
                        stab a,x
                        dec Cont_TCL
                        bra exit_tcl_est4
UltimaTecla_EsEnter
                        clr Cont_TCL
                        bset Banderas_1,Array_OK
                        bra exit_tcl_est4

exit_tcl_est4
                        movb #$FF,Tecla
                        movw #TareaTCL_Est1,Est_Pres_TCL

; ---------------------------------------------------------



; ==============================================================================
; ============================= Subrutina Bin_BCD_MuxP =========================
; ==============================================================================
Bin_BCD_MuxP
                pshd
                pshy
                pshx
                clr BCD
                ldab #0
                ldx #7

loop_bintobcd
                asla
                rol BCD
                pshx
                psha
check_magnitude
                ldaa BCD
                anda #$0F
                cmpa #$05
mayora5
                blt mayora5_end
                adda #$03
mayora5_end
                tab
                ldaa BCD
                anda #$F0
                cmpa #$50
mayora50
                blt mayora50_end
                adda #$30
mayora50_end
                aba
process
                staa BCD
                pula
                pulx
                dex
                cpx #0
                bne loop_bintobcd
                asla
                rol BCD
                pulx
                puly
                puld



                rts

; ==============================================================================
; ============================== Subrutina BCD-7Seg ============================
; ==============================================================================
BCD_7Seg
                ;Segment         dB $3F,$06,$5B,$4F,$66,$6D,$7D,$07,$7F,$6F
                ; aa = 99
                ldx #Segment
                ldaa BCD1
                psha
                anda #$F0
                lsra
                lsra
                lsra
                lsra
                ldab a,x
                stab Dsp1 ; decenas BCD1 en DISP1

                pula
                anda #$0F
                ldab a,x
                stab Dsp2 ; unidades BCD1 en DISP2

                ldaa BCD2
                psha
                anda #$F0
                lsra
                lsra
                lsra
                lsra
                ldab a,x
                stab Dsp3 ; decenas BCD2 en DISP3

                pula
                anda #$0F
                ldab a,x
                stab Dsp4 ; unidades BCD2 en DISP4

                rts

; ==============================================================================
; ============================= Subrutina Leer Teclado =========================
; ==============================================================================
Leer_Teclado            movb #$EF,PATRON
                        ldx #Teclas
loop_leer_teclado       movb PATRON,PORTA
                        ;clrb
                        nop
                        nop
                        nop
                        brclr PORTA,$04,tcl_colu2
                        brclr PORTA,$02,tcl_colu1
                        brclr PORTA,$01,tcl_colu0
                        orcc #$01
                        rol PATRON
                        ldab PATRON
                        cmpb #$FF
                        bne loop_leer_teclado
                        ldaa #$FF
                        lbra exit_leer_teclado
tcl_colu1

        brclr PORTA,$20,dig5
        brclr PORTA,$40,dig8
        brclr PORTA,$80,dig0
        brclr PORTA,$10,dig2
        lbra exit_leer_teclado

tcl_colu0
        brclr PORTA,$10,dig1
        brclr PORTA,$20,dig4
        brclr PORTA,$40,dig7
        brclr PORTA,$80,digB
        lbra exit_leer_teclado

tcl_colu2
        brclr PORTA,$10,dig3
        brclr PORTA,$20,dig6
        brclr PORTA,$40,dig9
        brclr PORTA,$80,digE
        lbra exit_leer_teclado



; Teclas            dB $01,$02,$03,$04,$05,$06,$07,$08,$09,$0B,$00,$0E
dig1    clrb
        ldaa b,x
        staa Tecla
        bra exit_leer_teclado
dig4    ldab #3
        ldaa b,x
        staa Tecla
        bra exit_leer_teclado
dig7    ldab #6
        ldaa b,x
        staa Tecla
        bra exit_leer_teclado
digB    ldab #9
        ldaa b,x
        staa Tecla
        bra exit_leer_teclado

; Teclas            dB $01,$02,$03,$04,$05,$06,$07,$08,$09,$0B,$00,$0E
dig2    ldab #1
        ldaa b,x
        staa Tecla
        bra exit_leer_teclado
dig5    ldab #4
        ldaa b,x
        staa Tecla
        bra exit_leer_teclado
dig8    ldab #7
        ldaa b,x
        staa Tecla
        bra exit_leer_teclado
dig0    ldab #10
        ldaa b,x
        staa Tecla
        bra exit_leer_teclado

; -------------------------------------------

dig3    ldab #2
        ldaa b,x
        staa Tecla
        bra exit_leer_teclado

dig6    ldab #5
        ldaa b,x
        staa Tecla
        bra exit_leer_teclado

dig9    ldab #8
        ldaa b,x
        staa Tecla
        bra exit_leer_teclado

digE    ldab #11
        ldaa b,x
        staa Tecla


exit_leer_teclado       rts




;******************************************************************************
;                       SUBRUTINA DE ATENCION A RTI
;******************************************************************************

Maquina_Tiempos:
               ;{COLOCAR EL CODIGO DE LA SUBRUTINA QUE IMPLEMENTA LA
               ; MAQUINA DE TIEMPOS }
               ;******************************************************************************
               ldx #Tabla_Timers_BaseT

               jsr Decre_Timers_BaseT

               ldd Timer1mS
               cpd #0
               bne check_10ms

               Movw #tTimer1mS,Timer1mS
               ldx #Tabla_Timers_Base1mS

               jsr Decre_Timers
check_10ms
               ;tst Timer10mS
               ldd Timer10mS
               cpd #0
               bne check_100ms

               Movw #tTimer10mS,Timer10mS
               ldx #Tabla_Timers_Base10mS

               jsr Decre_Timers

check_100ms
               ;tst Timer100mS
               ldd Timer100mS
               cpd #0
               bne check_1s

               Movw #tTimer100mS,Timer100mS
               ldx #Tabla_Timers_Base100mS

               jsr Decre_Timers

check_1s
              ;tst Timer1S
              ldd Timer1S
              cpd #0
              bne exit_checking

              Movw #tTimer1S,Timer1S
              ldx #Tabla_Timers_Base1S

              jsr Decre_Timers

exit_checking
               ;bset CRGFLG,#$80 ; RTIF = 1
               ldd TCNT
               addd #480
               std TC5
               RTI

; ==============================================================================
; ===== Subrutina de Maquina_Tiempos, Decre_Timers_BaseT
; ==============================================================================
Decre_Timers_BaseT
                ldy 2,x+
                cpy #0
                beq Decre_Timers_BaseT
                cpy #$FFFF
                bne dec_timer
                rts
dec_timer
                dey
                sty -2,x
                bra Decre_Timers_BaseT

; ==============================================================================
; ===== Subrutina de Maquina_Tiempos, Decre_Timers
; ==============================================================================

Decre_Timers
                ldaa 0,x
                cmpa #0
                beq inc_decre_timers
                cmpa #$FF
                beq exit_decre_timers


load_decre_timers
                dec 0,x
inc_decre_timers

                inx
                bra Decre_Timers
exit_decre_timers
                rts
