;******************************************************************************
 ;***           Marvin Castro Castro - C01884 Microprocesadores - EIE UCR
 ;***                   PROYECTO FINAL: RADAR 623
 ;*** ==========================================================================
 ;    Descripción: RADAR 623 Es un radar de control de velocidad de vehículos.
 ;    Por medio de dos botones (PH3 Y PH0) se simulan dos sensores ultrasónicos
 ;    y sacando la diferencia de tiempos entre estos se puede calcular la 
 ;    velocidad del vehículo. Si el vehiculo supera la velocidad límite se 
 ;    activa una alarma. Si la velocidad es menor o mayor a lo soportado por el
 ;    sensor, se comunicará por medio de la pantalla. Si la velocidad se 
 ;    encuentra en el rango aceptado, se mostrará la velocidad del vehículo y 
 ;    la velocidad límite por la cantidad de tiempo que le toma al vehículo
 ;    llegar a la pantalla. 
 ;  
 ;    Por medio de los DIP SWITCHES (PH7 y PH6) se escoje el modo de operación 
 ;    de la aplicación, siendo los modos: inactivo, configuración, y en servicio. 

 ;******************************************************************************
#include registers.inc
 ;******************************************************************************
 ;                 RELOCALIZACIÓN DE VECTOR DE INTERRUPCIÓN
 ;******************************************************************************
                                Org $3E64       ; vector Output Compare
                                dw Maquina_Tiempos

;******************************************************************************
;                   DECLARACIÓN DE LOS VALORES DEL PROGRAMA
;******************************************************************************

; ------------------------------ TAREA TECLADO ---------------------------------
tSuprRebTCL             EQU 20  ; Supr. rebotes teclado matricial  (20 ms)       

; ---------------------------- TAREA PANTALLA_MUX ------------------------------
tTimerDigito            EQU 2   ; Tiempo de digito de 2 ms, pantalla 7 segmentos 
MaxCountTicks           EQU 100 ; Contador para la tarea de Brillo en la pantalla
                                ; de 7 segmentos. 


; -------------------------------- TAREA LCD  ----------------------------------

; --------- Retardos para la sincronización estroboscópica de la pantalla LCD
tTimer260uS:            EQU 13      ; 260/20 = 13    
tTimer40uS:             EQU 2       ; 40/20 = 2    
tTimer2mS:              EQU 2       ; Se usa el timer de 1ms
EOB                     EQU $FF     ; End Of Block
Clear_LCD               EQU $01     ; Comando de Clear (LCD)
ADD_L1                  EQU $80     ; Dirección de la primera línea    
ADD_L2                  EQU $C0     ; Dirección de la segunda línea  

; ------------------------- TAREAS LEER_PB1, LEER_PB0 --------------------------

PortPB                     EQU PTIH ; Puerto de los pushbuttons
tSupRebPB0                 EQU 10   ; Supr. rebotes para el botón PH0 (10 ms)
tSupRebPB1                 EQU 10   ; Supr. rebotes para el botón PH3 (10 ms)
tShortP0                   EQU 25   ; Tiempo de short press, PH0 (25 ms)
tShortP1                   EQU 25   ; Tiempo de short press, PH1 (25 ms)
tLongP0                    EQU 3    ; Tiempo de long press, PH0 (3 s)
tLongP1                    EQU 3    ; Tiempo de long press, PH1 (3 ms)
MaskPB1                    EQU $04  ; TODO: Máscara para el botón PH2
MaskPB0                    EQU $01  ; Máscara para el botón PH0

; ---------------------------- TAREA CONFIGURAR --------------------------------

LDConfig                 EQU $02    ; Led posición dos
LimMax                   EQU 90     ; Límite de velocidad máximo
LimMin                   EQU 65     ; Límite de velocidad mínimo

; ---------------------------- TAREA INACTIVO ----------------------------------
LDInac                   EQU $01    ; Led posición uno
tTimerVLim               EQU 30     ; Tiempo para el TimerVLim (30 * 100 ms)

; --------------------------- TAREA EnServicio ---------------------------------

LDEnServ                  EQU $04   ; Led posición tres
tTimerVel                 EQU 100   ; Timer para el cálculo de velocidad (10 * 100 ms)
tTimerError               EQU 3     ; Timer de error (3 s)
VelocMin                  EQU 45    ; Velocidad mínima aceptable por el sensor
VelocMax                  EQU 99    ; Velocidad máxima aceptable por el sensor

; ------------------------------ TAREA Brillo ----------------------------------
tTimerBrillo               EQU 4    ; Timer para el convertidor ATD (400 ms)
MaskSCF                    EQU $80  ; Máscara para el bit SCF del registro 
                                    ; ATD0STAT0

; ------------------------------ TAREA LeerDS ----------------------------------
tTimerRebDS               EQU 10    ; Supr. Rebotes DIP SWITCHES (10 ms)

; --------------------------- TAREA DesplazarLeds ------------------------------
tTimerDplzLeds            EQU 1     ; Cadencia de desplazamiento de LEDS 
                                    ; (100 ms)

; ------------------------------- Banderas -------------------------------------
; ----------  Banderas_1
ShortP0                   EQU $01   ; flag de Short press para el PH0
LongP0                    EQU $02   ; flag de Long press para el PH0
ShortP1                   EQU $04   ; flag de Short press para el PH1
LongP1                    EQU $08   ; flag de Long press para el PH1
Array_OK                  EQU $10   ; flag de finalización de escritura con el 
                                    ; teclado matricial
; ---------- Banderas_2
RS                        EQU $01   ; flag de LCD: indica si se transmite un dato o no
LCD_OK                    EQU $02   ; flag de finalización de mensajes del LCD
FinSendLCD                EQU $04   ; flag que indica envío de un byte en LCD
SecondLine                EQU $08   ; flag que indica que se va a transmitir la segunda línea
Alarma                    EQU $10   ; flag que indica la alarma activa
DsplzIzquierda            EQU $20   ; flag que indica que los leds van de izq a derecha

; -------------------------------- GENERALES -----------------------------------
tTimerLDTst               EQU 5     ; Tiempo de parpadeo de LED testigo (500 ms)

tTimer1mS:                EQU 50     ;Base de tiempo de 1 mS (1 ms x 1 / 20us)
tTimer10mS:               EQU 500    ;Base de tiempo de 10 mS (1 mS x 10 / 20us)
tTimer100mS:              EQU 5000   ;Base de tiempo de 100 mS (10 mS x 100 / 20us)
tTimer1S:                 EQU 50000  ;Base de tiempo de 1 segundo (100 mS x 10 / 20us)

;******************************************************************************
;                   DECLARACIÓN DE LAS ESTRUCTURAS DE DATOS
;******************************************************************************
                                Org $1000

; ----------------------------- TAREA_TECLADO ----------------------------------
;                          Dirección (hex) || Descripción
MAX_TCL                 db 5         ;1000    Cantidad máxima de carácteres en el teclado
Tecla                   db $FF       ;1001    Tecla inicial
Tecla_IN                db $FF       ;1002    Tecla posterior - supresión de rebotes
Cont_TCL                db $00       ;1003    Cantidad de teclas ingresadas
Patron                  db $00       ;1004    Patrón a desplazar para la lectura del teclado
Est_Pres_TCL            ds 2         ;1006    Estado presente, máquina estados TCL
                                org $1010
Num_Array               db $FF,$FF,$FF,$FF,$FF ;1010 Array contenedor de 
                                               ; carácteres del teclado matricial

; ---------------------------- TAREA PANTALLA_MUX ------------------------------
                                org $1020
EstPres_PantallaMUX     ds 2       ;1021  Estado presente, máquina estados pmux  
Dsp1                    ds 1       ;1022  Variable a multiplexar en el primer display 
Dsp2                    ds 1       ;1023  Variable a multiplexar en el segundo display 
Dsp3                    ds 1       ;1024  Variable a multiplexar en el tercer display 
Dsp4                    ds 1       ;1025  Variable a multiplexar en el cuarto display 
LEDS                    ds 1       ;1026  Variable a colocar en los LEDS 
Cont_Dig                ds 1       ;1027  Contador de digito a multiplexar en 7 Seg  
Brillo                  db 100     ;1028  Variable de brillo en los 7 segmentos 
                                   ;      (100 brillo máximo, 0 brillo mínimo)


; --------------------------- Subrutina CONVERSION  ----------------------------
BCD                     ds 1  ;1029   Variable resultado de Bin_BCD_MuxP
Cont_BCD                ds 1  ;102A   Variable de contador Bin_BCD_MuxP
BCD1                    ds 1  ;102B   Variable en BCD a poner en Dsp1 y Dsp2 
BCD2                    ds 1  ;102C   Variable en BCD a poner en Dsp3 y Dsp4

; -------------------------------- TAREA LCD  ----------------------------------
IniDsp                  dB $28,$28,$06,$0C,$FF ;1031 Tabla con los comandos para
                        ; inicializar el LCD
Punt_LCD                ds 2  ;1033   Puntero que recorre los elementos a enviar
CharLCD                 ds 1  ;1034   Contiene el byte a enviar
Msg_L1                  ds 2  ;1036   Puntero con direc. del mensaje a poner en linea1
Msg_L2                  ds 2  ;1038   Puntero con direc. del mensaje a poner en linea2
EstPres_SendLCD         ds 2  ;103A   Estado presente, máquina estados SendLCD  
EstPres_TareaLCD        ds 2  ;$103C  Estado presente, máquina estados LCD

; ------------------------- TAREAS LEER_PB1, LEER_PB0 --------------------------
Est_Pres_LeerPB0           ds 2       ;103E Estado presente, máquina estados PB0
Est_Pres_LeerPB1           ds 2       ;1040 Estado presente, máquina estados PB1

; ---------------------------- TAREA CONFIGURAR --------------------------------
Est_Pres_TConfig          ds 2        ;1042 Estado presente, máquina estados ModoConfig
ValorLIM                  ds 1        ;1043 Variable temporal
Vel_LIM                   dB 65       ;1044 Variable de velocidad límite 

; ---------------------------- TAREA INACTIVO ---------------------------------
Est_Pres_TInac            ds 2        ;1046 Estado presente, máquina estados ModoInact

; ----------------------------- TAREA EnServicio -------------------------------
Est_Pres_TServ            ds 2        ;1048 Estado presente, máquina estados EnServ
Vel_Calc                  ds 1        ;1049 Velocidad Calculada 
DeltaT                    ds 1        ;104A Diferencia de tiempo entre sensor1 y 2

; ------------------------------ TAREA Brillo ----------------------------------
Est_Pres_TBrillo           ds 2       ;104C Estado presente, máquina estados Brillo

; ------------------------------ TAREA LeerDS ----------------------------------
Est_Pres_LeerDS            ds 2       ;104E Estado presente, máquina estados LeerDS
Temp_DS                    ds 1       ;104F Variable temporal DIPSWITCHES
Valor_DS                   ds 1       ;1050 Variable DIPSWITCHES con rebotes suprimidos

; --------------------------- TAREA DesplazarLeds ------------------------------
Est_Pres_DsplzLeds         ds 2       ;1052 Estado presente, máquina estados DsplzLeds
DplzLeds                   ds 1       ;1053 Valor de máscara a ser aplicada en LEDS

; -------------------------------- BANDERAS ------------------------------------
                                org $1070
Banderas_1                ds 1  
Banderas_2                ds 1

; -------------------------------- GENERALES -----------------------------------
                                org $1080
LED_Testigo               ds 1          ; Variable con el valor a poner en PTP7-4

; --------------------------------- TABLAS -------------------------------------
                                    org $1100
;-------------------------------------------------------------------------------
;                              TABLA DE SEGMENT
                ;    0,  1,  2,  3,  4,  5,  6,  7,  8,  9,  -
Segment         dB $3F,$06,$5B,$4F,$66,$6D,$7D,$07,$7F,$6F,$40
;-------------------------------------------------------------------------------
;                              TABLA DE TECLAS

Teclas          dB $01,$02,$03,$04,$05,$06,$07,$08,$09,$0B,$00,$0E

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
MSG_ENSERV_WAIT FCC "   ESPERANDO... "
                db EOB
MSG_RADAR623    FCC "   RADAR  623   "
                db EOB
MSG_ENSERV_CALC FCC "  CALCULANDO... "
                db EOB
MSG_ENSERV_ERR1 FCC " ** VELOCIDAD **"
                db EOB
MSG_ENSERV_ERR2 FCC "*FUERA DE RANGO*"
                db EOB
MSG_ENSERV_ALR1 FCC "*V_LIM EXCEDIDA*"
                db EOB
MSG_ENSERV_INDI FCC "V_LIM   SU_VELOC"
                db EOB
MSG_ENSERV1     FCC "MODO EN SERVICIO"
                db EOB
MSG_INACTIV     FCC "  MODO INACTIVO "
                db EOB



;===============================================================================
;                              TABLA DE TIMERS
;===============================================================================
                                Org $1500
Tabla_Timers_BaseT:

Timer1mS                ds 2       ;1501 Timers por defecto de la máquina de tiempos
Timer10mS:              ds 2       ;1503
Timer100mS:             ds 2       ;1505
Timer1S:                ds 2       ;1507
Timer260uS:             ds 2       ;1509 Timer 260uS para LCD
Timer40uS:              ds 2       ;150B Timer 40uS para LCD

Counter_Ticks           ds 2       ;150D Contador para el brillo de 7 Segmentos
Fin_BaseT               dW $FFFF   ;150F

Tabla_Timers_Base1mS

Timer_Digito            ds 1       ;1510 Timer para cada digito de 7 Segmentos
Timer_RebPB0            ds 1       ;1511 Supresión de rebotes PB0
Timer_RebPB1            ds 1       ;1512 Supresión de rebotes PB1
Timer_RebTCL            ds 1       ;1513 Supresión de rebotes TCL
Timer2mS                ds 1       ;1514 Timer 2 ms para LCD
TimerRebDS              ds 1       ;1515 Supresión de rebotes DS

Fin_Base1mS:            dB $FF     ;1516

Tabla_Timers_Base10mS

Timer_SHP0              ds 1       ;1517 Short press PB0
Timer_SHP1              ds 1       ;1518 Short press PB1

Fin_Base10ms            dB $FF     ; 1519

Tabla_Timers_Base100mS

Timer1_100mS              ds 1     ;151A    
Timer_LED_Testigo         ds 1     ;151B Timer para parpadeo de led testigo
TimerBrillo               ds 1     ;151C Timer para tarea de brillo
TimerVel                  ds 1     ;151D Timer para el calculo de velocidad EnServicio
TimerPant                 ds 1     ;151E Timer para que el vehiculo recorra 200m
TimerFinPant              ds 1     ;151F Timer para que el vehiculo recorra 300m
TimerDplzLeds             ds 1     ;1520 Timer para cadencia de leds

Fin_Base100mS           dB $FF     ;1521

Tabla_Timers_Base1S

Timer_LP0                  ds 1    ;1522 Long Press PB0
Timer_LP1                  ds 1    ;1523 Long Press PB1
TimerError                 ds 1    ; Timer de error
TimerInac                  ds 1    ; Timer para display en estado inactivo

Fin_Base1S                dB $FF

;===============================================================================
;                           CONFIGURACIÓN DE PERIFERICOS
;===============================================================================
                              Org $2000

; ------------------------- Inicialización de timers ---------------------------
        movw #tTimer1mS,Timer1mS
        movw #tTimer10mS,Timer10mS         
        movw #tTimer100mS,Timer100mS
        movw #tTimer1S,Timer1S
        movb #tTimerDigito,Timer_Digito
        movb #tTimerLDTst,Timer_LED_Testigo  
        movb #1,Cont_Dig

; -------------------- Inicialización de Máquinas de estado ---------------------
        movw #TareaLeerDS_Est1,Est_Pres_LeerDS
        movw #TareaConfig_Est1,Est_Pres_TConfig
        movw #DsplzLeds_Est1,Est_Pres_DsplzLeds
        movw #TareaServ_Est1,Est_Pres_TServ
        movw #LeerPB0_Est1,Est_Pres_LeerPB0
        movw #LeerPB1_Est1,Est_Pres_LeerPB1
        movw #TareaTCL_Est1,Est_Pres_TCL
        movw #TareaMuxPantalla_Est1,EstPres_PantallaMUX
        movw #TareaSendLCD_Est1,EstPres_SendLCD
        movw #TareaLCD_Est1,EstPres_TareaLCD
        movw #TareaBrillo_Est1,Est_Pres_TBrillo

; ------------------------ Bloque de configuración de LEDS ---------------------
        Bset DDRB,$FF       ; Habilitar todos los LEDS
        Bset DDRJ,$02       ; Poner en bajo PTJ.1 para funcionamiento de LEDS
        BClr PTJ,$02

; --------------Bloque de configuración de Disp. 7 Segmentos y Led RGB----------
        Movb #$7F,DDRP      ; Habilito (pongo como salidas) PP6-PP4 (RGB LED)
                            ; y PP3-PP0 (digitos de 7 segmentos)
        movb #$0F,PTP           

; -------------- Bloque de configuración de puerto de botones y DS ----------------
        movb #$00,DDRH      ; Puerto H como entradas para botones y DIP SWITCHES 

; ---------------- Bloque de configuración de convertidor ATD ------------------
        movb #$20,ATD0CTL3  ; SNC = 2; 2 conversiones por secuencia
        movb #$C8,ATD0CTL4  ; SRES8 = 1 (8 bits)  TODO: SMP1 = 0, SMP0 = 1, 4 Ciclos 
        movb #$07,ATD0CTL5  ; CC=CB=CA=1, canal 7 del ATD
; ---------------- Bloque de configuración Output Compare Canal 5 ------------
        Movb #$90,TSCR1     ; TEN = 1 (Habilitar TIMER), TFFCA = 1 (Limpiar banderas)
        Movb #$00,TSCR2     ; TOI = 0, PRS = 0 (SIN PREESCALADOR)
        Movb #$20,TIOS      ; IOS5 = 1, canal 5 como output compare
        Movb #$20,TIE       ; C5I = 1, interrupción del canal 5

        ldd TCNT
        addd #480           ; Valor de TCn para un periodo de 20 us
        std TC5
; ---------------- Bloque de configuración Teclado Matricial -------------------
        movb #$F0,DDRA      ; pongo como salidas los 4 msb
        movb #$01,PUCR      ; activo pull ups para puerto A
        
;===============================================================================
;                           PROGRAMA PRINCIPAL
;===============================================================================
        Lds #$3BFF          ; Inicialización de puntero de pila
        Cli                 ; habilitación de interrupciones
        Clr Banderas_1      ; limpieza de variables en memoria
        Clr Banderas_2
        Clr LEDS
        Clr Msg_L1
        CLR Msg_L2
        clr Punt_LCD
        clr CharLCD
        clr Cont_TCL

        jsr Rutina_InitLCD  ; 
        ;movw #MSG1,Msg_L1
        ;movw #MSG2,Msg_L2
        ;bclr Banderas_2,LCD_OK

;  ============================  Despachador de Tareas ==========================
Despachador_Tareas
; ----- Mensajes LCD ------ ;
        brset Banderas_2,LCD_OK,skipLCD     ; Si LCD_OK = 0, se ejecuta Tarea_LCD
        jsr Tarea_LCD
skipLCD

        Jsr Tarea_Led_Testigo
        jsr Tarea_Teclado
        Jsr Tarea_LeerPB0
        Jsr Tarea_LeerPB1
        jsr Tarea_LeerDS
        jsr ControlModo
        jsr Tarea_Mux_Pantalla
        jsr Tarea_Brillo
        jsr Tarea_DsplzLeds
        Bra Despachador_Tareas

;******************************************************************************
;                              TAREA ModoInactivo
;******************************************************************************
;   Esta tarea corresponde al modo inactivo de la aplicación de RADAR 623. Se
;   encarga de desplegar el mensaje de Modo Inactivo en la pantalla de LCD. 
;   Si se presiona PB1 o PB0 como LongPress se muestra el límite de velocidad 
;   en los Display de 7 segmentos de la derecha durante un periodo de 3 segundos.  
Tarea_ModoInactivo
                ldx Est_Pres_TInac
                jsr 0,x
                rts
;============================ ModoInactivo ESTADO 1 ===============================
;   En el primer estado se coloca el mensaje de modo inactivo en el LCD, y 
;   también apaga los display de 7 segmentos. Siempre pasa al segundo estado. 
TInac_Est1
                movw #MSG_RADAR623,Msg_L1  ;Colocación de mensajes en los punteros 
                movw #MSG_INACTIV,Msg_L2
                bclr Banderas_2,LCD_OK  ; borrar para correr LCD
                
                bset DDRP,$0F           ; Apagar displays de 7 segmentos
                bset PTP,$0F
                movw #TInac_Est2,Est_Pres_TInac
                rts
;============================ ModoInactivo ESTADO 2 ===============================
;   Si se detecta un LongPress en alguno de los botones, se coloca la velocidad 
;   límite en los display de 7 segmentos de la derecha.  
TInac_Est2
                brset Banderas_1,LongP0,mostrarInactivo     
                brset Banderas_1,LongP1,mostrarInactivo
                bset DDRP,$0F       
                bset PTP,$0F    
                bra ret_TInac_est2
mostrarInactivo                                 
                movb #tTimerVLim,TimerDplzLeds      ; cargar timer de 30*100 ms
                ldaa Vel_LIM                        ; Se utilza TimerDplzLeds
                jsr Bin_BCD_MuxP                    ; porque es inactivo en esta tarea
                movb BCD,BCD2                       ; Colocar velocidad límite
                jsr BCD_7Seg                        ; en 7 Dsp3, Dsp4
                movw #TInac_Est3,Est_Pres_TInac     
                ;bra just_ret_tinac2

ret_TInac_est2

                 rts
;============================ ModoInactivo ESTADO 3 ===============================
;   En el estado 3 se espera a que se acabe el timer TimerDplzLeds, que indica que 
;   ya no se debe mostrar la velocidad límite en los 7 segmentos. 
TInac_Est3
                tst TimerDplzLeds
                bne ret_TInac_est3
                
                movw #TInac_Est1,Est_Pres_TInac
                bclr Banderas_1,LongP0          ; Limpiar banderas para que pueda
                bclr Banderas_1,LongP1          ; ejecutarse de nuevo
ret_TInac_est3  bset DDRP,$03                   ; Habilitar 2 displays de la
                bset PTP,$03                    ; derecha
                rts
;******************************************************************************
;                               TAREA DsplzLeds
;******************************************************************************
;   Esta tarea se encarga de desplazar los cinco leds MSB de izquierda a derecha,
;   un led a la vez. Utiliza la bandera DsplzIzquierda para saber si está
;   recorriendo de izquiera a derecha o viceversa. 
Tarea_DsplzLeds
                ldx Est_Pres_DsplzLeds
                jsr 0,x
       rts

;============================ DsplzLeds ESTADO 1 ===============================
;   En el primer estado inicializa los timers, la variable de DplzLeds y el 
;   próximo estado. DplzLeds contiene el patrón de LEDS. 
DsplzLeds_Est1
                brclr Banderas_2,Alarma,retDsplzLeds_est1
                movw #DsplzLeds_Est2,Est_Pres_DsplzLeds
                movb #tTimerDplzLeds,TimerDplzLeds
                movb #$80,DplzLeds
                

retDsplzLeds_est1
                rts
;========================== DsplzLeds ESTADO 1 ================================
;   En el estado 2 se comprueba el valor actual del patrón de LEDS. Si se 
;   determina que llegó al final de la respectiva orientación, cambia el valor
;   de la bandera, para recorrer el patrón de la manera inversa. 
DsplzLeds_Est2
                tst TimerDplzLeds
                bne retDsplzLeds_est2
                brset Banderas_2,DsplzIzquierda,IzqADerecha
                ; desplazamiento derecha a izquierda
                ldaa DplzLeds
                cmpa #$80       ; Llegó al final de la izquierda?
                beq cambiarAIzq
                lsl DplzLeds    ; No llegó al final, desplazar a la izquierda
                bra dplz2_join
cambiarAIzq     ; llegó al final de la izquierda - cambiar a izq-derecha
                bset Banderas_2,DsplzIzquierda
                bra dplz2_join
IzqADerecha
                ldaa DplzLeds
                cmpa #$08      ; llegó al final de la derecha?
                beq cambiarADer 
                lsr DplzLeds    ; No llegó al final, desplazar a la derecha
                bra dplz2_join
cambiarADer     ; llegó al final de la izquierda - cambiar a der-izq
                bclr Banderas_2,DsplzIzquierda 

dplz2_join      movb DplzLeds,LEDS ; se carga el valor a los LEDS

                brset Banderas_2,Alarma,reloadLedTimer
                movw #DsplzLeds_Est1,Est_Pres_DsplzLeds
                bclr LEDS,$80
                bra retDsplzLeds_est2
                
reloadLedTimer  ; se recarga el timer si Alarma sigue activa
                movb #tTimerDplzLeds,TimerDplzLeds
retDsplzLeds_est2
                rts
;******************************************************************************
;                               TAREA EnServicio
;******************************************************************************
;   EnServicio es la tarea de RADAR623 que realiza los cálculos y despliega la
;   información relevante. Primeramente, carga los mensajes respectivos en 
;   la pantalla LCD. Luego espera a que el usuario presione el botón de más
;   a la izquierda (PB1) indicando que el vehículo pasó por el primer sensor.
;   Luego espera a que el botón de más a la derecha sea presionado, y con la
;   diferencia de tiempos entre los botones presionados, llama a la subrutina 
;   Calcula, que se encarga de calcular las siguientes variables:
;           -DeltaT (diferencia de tiempos entre sensor 1 y 2, décimas de segundo)
;           -Vel_Calc (Velocidad del vehículo en km/h)
;           -TimerPant (tiempo que le toma estar a 100 metros de la pantalla, 
;                      (décimas de segundo)
;           -TimerFinPant (tiempo que le toma alcanzar la pantalla)
;   
;   Esta tarea además, despliega información en los 7 segmentos y compara
;   la velocidad calculada con el límite de velocidad, si supera el límite
;   activa una alarma. 
Tarea_EnServicio
                ldx Est_Pres_TServ
                jsr 0,x
       rts

;========================== EnServicio ESTADO 1 ================================
;   Se cargan los mensajes en la pantalla LCD y se apagan los 7 segmentos.
;   Pasa siempre al próximo estado.
TareaServ_Est1  movw #MSG_RADAR623,Msg_L1
                movw #MSG_ENSERV_WAIT,Msg_L2
                bclr Banderas_2,LCD_OK

                movb #$00,BCD1  ;TODO
                movb #$00,BCD2  ;TODO
                jsr BCD_7Seg
                
                bset DDRP,$0F
                bset PTP,$0F
                
                movw #TareaServ_Est2,Est_Pres_TServ
                rts
;========================== EnServicio ESTADO 2 ================================
;   En este estado se espera a que el usuario presione el PB1. Una vez presionado
;   inicializa el timer de TimerVel y pasa al estado 3. 
TareaServ_Est2
                brclr Banderas_1,ShortP1,retEnServ_est2

                movw #MSG_RADAR623,Msg_L1
                movw #MSG_ENSERV_CALC,Msg_L2
                bclr Banderas_2,LCD_OK
                
                bclr Banderas_1,ShortP1
                ;bset DDRP,$0F
                ;bset PTP,$0F
                
                movb #tTimerVel,TimerVel
                movw #TareaServ_Est3,Est_Pres_TServ

retEnServ_est2  bset DDRP,$0F
                bset PTP,$0F
		rts

;========================== EnServicio ESTADO 3 ================================
;   En el estado 3 se espera a que el usuario presione el botón PB0. Una vez 
;   presionado, llama a la subrutina Calcula la cual se encarga de obtener las
;   variables mencionadas en el encabezado. Si la velocidad calculada se 
;   encuentra en el rango permitido (entre 45 y 99 km/h) se pasa al próximo
;   estado. 
TareaServ_Est3
                brclr Banderas_1,ShortP0,retEnServ_est3
                ;movb #$55,BCD2
                ;jsr BCD_7Seg
                movb #$80,Dsp4  ;TODO
                movb #$80,Dsp3  ;TODO
                jsr Calcula
                nop         
                nop
                ldaa Vel_Calc   
                ;cmpa #20
                ; verificar si la velocidad se encuentra en el rango aceptable
                cmpa #VelocMin      ; mayor a 45 km/h?
                bls est3_go_error
                cmpa #VelocMax      ; menor a 99 km/h?
                ;cmpa #200
                bhs est3_go_error
                movw #TareaServ_Est4,Est_Pres_TServ ; si se encuentra en el rango
                                                    ; pasa al estado 4. 

                bra retEnServ_est3
;   Si la velocidad no se encuentra en el rango permitido, se coloca el mensaje de 
;   error y se ponen guiones en los 7 segmentos.                 
est3_go_error
                movw #MSG_ENSERV_ERR1,Msg_L1
                movw #MSG_ENSERV_ERR2,Msg_L2
                bclr Banderas_2,LCD_OK
                movb #$AA,BCD1  ; valores para guiones en 7 segmentos. 
                movb #$AA,BCD2

                jsr BCD_7Seg
                movb #tTimerError,TimerError    ; carga de timer de error
                movw #TareaServ_Est5,Est_Pres_TServ ; y se pasa al estado 5
retEnServ_est3
		        bset DDRP,$0F
                bset PTP,$0F                ; apagar 7 segmentos 
                bclr Banderas_1,ShortP0     ; borrar banderas de ShortP
                bclr Banderas_1,ShortP1     ; para ser reutilizable 
		rts
;========================== EnServicio ESTADO 4 ================================
;   Se accede al estado 4 cuando la velocidad calculada se encuentra en los 
;   límites apropiados del radar, es decir, cuando se calcula que la velocidad 
;   del vehículo está entre 45 y 99 km/h.
;   El estado 4 verifica si la velocidad calculada es menor a la velocidad 
;   límite. De ser menor, el sistema desplega a la izquierda en los 7 segmentos
;   la velocidad límite y a la derecha la velocidad del vehículo. 
;   En caso de que la velocidad del vehículo sea mayor a la límite, hace lo mismo
;   pero también activa una alarma durante 3 segundos.
;   El despliegue de información ocurre cuando el TimerPant se hace igual a 0.
;   Esto quiere decir que el vehículo se encuentra a 100 metros de la pantalla.  
TareaServ_Est4
                tst TimerPant
                bne retEnServ_est4
                ldaa Vel_LIM
                jsr Bin_BCD_MuxP   ; colocación de la velocidad límite a la izq 
                movb BCD,BCD1
                ldaa Vel_Calc
                jsr Bin_BCD_MuxP   ; colocación de la velocidad calculada a la der
                movb BCD,BCD2
                jsr BCD_7Seg    
                ldaa Vel_Calc
                cmpa Vel_LIM
                bhi es4_go_alarm    ; si es mayor, activar alarma
                movw #MSG_ENSERV1,Msg_L1    ; si es menor, desplegar info normal
                movw #MSG_ENSERV_INDI,Msg_L2
                bclr Banderas_2,LCD_OK
                bra  EnServ4_to6
                
es4_go_alarm    ; solo si la velocidad del vehiculo supera el límite
                movw #MSG_ENSERV_ALR1,Msg_L1
                movw #MSG_ENSERV_INDI,Msg_L2
                bclr Banderas_2,LCD_OK
                bset Banderas_2,Alarma
                
EnServ4_to6     movw #TareaServ_Est6,Est_Pres_TServ
retEnServ_est4
		        bset DDRP,$03
                bset PTP,$03
		        rts

;========================== EnServicio ESTADO 5 ================================
;   Este estado solo se accede cuando la velocidad del vehículo no se encuentra
;   entre los límites del sensor (45 hasta 99 km/h). En este estado se espera a
;   que se acabe el timer de error, para que se deje de desplegar el mensaje de 
;   error y pase al estado inicial.
TareaServ_Est5
                tst TimerError
                bne retEnServ_est5
                
                movw #TareaServ_Est1,Est_Pres_TServ
retEnServ_est5  rts

;========================== EnServicio ESTADO 6 ================================
;   Este estado ocurre cuando la velocidad del vehículo se encuentra entre los 
;   límites del sensor. Se espera a que se acabe el TimerFinPant, que indica
;   que el vehículo alcanzó a la pantalla, indicando que ya debe dejar de 
;   desplegar información en el LCD y en los 7 segmentos, por lo tanto,
;   se pasa al estado 1. 
TareaServ_Est6
                tst TimerFinPant
                bne retEnServ_est6
                bclr Banderas_2,Alarma
                
                movw #TareaServ_Est1,Est_Pres_TServ
retEnServ_est6  rts
;******************************************************************************
;                               SUBRUTINA CALCULA
;******************************************************************************
;   CALCULA se encarga de calcular las variables necesarias para la tarea
;   EnServicio del RADAR623.
;   Recibe: El valor del timer TimerVel 
;   Entrega: las siguientes variables 
;           -DeltaT (diferencia de tiempos entre sensor 1 y 2, décimas de segundo)
;           -Vel_Calc (Velocidad del vehículo en km/h)
;           -TimerPant (tiempo que le toma estar a 100 metros de la pantalla, 
;                      (décimas de segundo)
;           -TimerFinPant (tiempo que le toma alcanzar la pantalla) 
;   Las ecuaciones implementadas son las siguientes. El por qué de las mismas se 
;   detalla en el reporte de esta entrega. 
;
;           DeltaT = 100 - TimerVel
;
;           Vel_Calc = 1440/DeltaT
;
;           TimerPant = 200/((Vel_Calc*10)/36)  = 200/(Vel_Calc/3.6)
;
;           TimerFinPant = 300/((Vel_Calc*10)/36)  = 300/(Vel_Calc/3.6)
Calcula
                ldaa #100
                ldab TimerVel
                sba
                staa DeltaT ; DeltaT = 100 - TimerVel
                

                tab   
                clra       ; muevo DeltaT a parte baja y limpio parte alta
                xgdx       ; muevo DeltaT a X como divisor
                ldd #1440  ; cargo dividiendo
                idiv       ; Vel_Calc = 1440/DeltaT
                xgdx       ; resultado desplazo a acumulador D
                stab Vel_Calc       ; guardo parte baja como Vel_Calc
                
                clra        
                ldab Vel_Calc       
                ldy #10
                emul            ; Vel_Calc * 10
                ldx #36
                idiv            ; Vel_Calc * 10/36
                pshx
                ldd #200        
                idiv            ; TimerPant = 200/(Vel_Calc * 10/36)
                xgdx            ; en segundos
                ldy #10         ; lo multiplico por 10 para ser consistente
                emul            ; con las unidades de TimerPant
                stab TimerPant
                
                pulx
                ldd #300    
                idiv
                xgdx        ; TimerPant = 300/(Vel_Calc * 10/36)
                ldy #10     ; lo multiplico por 10 para ser consistente
                emul        ; con las unidades de TimerPant
                stab TimerFinPant
                
                rts
                
;******************************************************************************
;                               TAREA MODO CONFIGURAR
;******************************************************************************
;   CONFIGURAR corresponde al modo de RADAR623 donde se puede modificar la 
;   velocidad límite del radar. Esta tarea hace uso de la subrutina de 
;   Leer_Teclado que se encarga de leer las teclas del teclado matricial.
;   y a su vez se ejecuta la Tarea Teclado que se encarga de conformar un 
;   arreglo de teclas válido. 
Tarea_Configurar
                ldx Est_Pres_TConfig
                jsr 0,x
       rts

;========================== CONFIGURAR ESTADO 1 ================================
;   Se cargan los mensajes en LCD y se apagan los displays de 7 segmentos de 
;   más a la izquierda.
;   Carga el valor de Vel_LIM, lo convierte de binario a bcd y lo pone en los 
;   displays de más a la derecha. También limpia el Num_Array por si hay valores
;   no previstos. 
TareaConfig_Est1
                ;bclr Banderas_2,SecondLine
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
;   Cuando se detecta la bandera Array_Ok, se convierte el valor de Num_Array
;   a binario y se compara con los valores de velocidades límites máximos 
;   y minimos para ver si es aceptable. Si se determina que se encuentra en el
;   rango, se guarda el nuevo valor en Vel_LIM, y se pasa nuevamente al estado 1. 

TareaConfig_Est2
                brclr Banderas_1,Array_OK,retConfig_est2
                jsr BCD_BIN
                cmpa #LimMin    ; 65 kmh
                blo est2_conf_borrartcl
                cmpa #LimMax    ; 90 kmh
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
                bclr Banderas_1,Array_OK
retConfig_est2  bset DDRP,$03
                bset PTP,$03
                rts

;******************************************************************************
;                               Subrutina Control Modo
;******************************************************************************
; Se encarga de seleccionar el modo de operación según los valores de los DIP
; Switches 7 y 6.

ControlModo
                brset Valor_DS,$80,Dip7on
                brset Valor_DS,$40,Dip6on
                jsr Tarea_ModoInactivo
                bset LEDS,LDInac
                bclr LEDS,LDConfig
                bclr LEDS,LDEnServ
                movw #TareaServ_Est1,Est_Pres_TServ
                movw #TareaConfig_Est1,Est_Pres_TConfig
                bra retControlModo
Dip6on          ; 0 1
                jsr Tarea_Configurar
                bclr LEDS,LDInac
                bset LEDS,LDConfig
                bclr LEDS,LDEnServ
                movw #TInac_Est1,Est_Pres_TInac
                movw #TareaServ_Est1,Est_Pres_TServ
                bra  retControlModo

Dip7on          brset Valor_DS,$40,Dip7_6on
                bra retControlModo

Dip7_6on        ; 1 1
                jsr Tarea_EnServicio
                bclr LEDS,LDInac
                bclr LEDS,LDConfig
                bset LEDS,LDEnServ
                movw #TareaConfig_Est1,Est_Pres_TConfig
                movw #TInac_Est1,Est_Pres_TInac

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
                        ;ldaa BIN1
                        ;jsr Bin_BCD_MuxP
                        ;movb BCD,BCD1



                        ;ldaa BIN2
                        ;jsr Bin_BCD_MuxP
                        ;movb BCD,BCD2



                        ;jsr BCD_7Seg

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
BCD_7Seg                                                                    ;-
                ;Segment         dB $3F,$06,$5B,$4F,$66,$6D,$7D,$07,$7F,$6F,$40
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
