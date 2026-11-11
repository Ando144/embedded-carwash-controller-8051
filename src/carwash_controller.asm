;==============================================================================
;  Real-Time Car Wash Tunnel Controller
;  Target    : Philips/NXP 80C552 (8051 core) @ 12 MHz
;  Toolchain : Keil uVision2 (A51 assembler) - simulated in the uVision debugger
;  Authors   : Andoni Ortiz de Zarate Urruela, Markel Valle Vivanco
;  Course    : Computer Architecture (Konputagailuen Arkitektura), UPV/EHU
;
;  Architecture
;    - 21-state finite state machine (E0..E20) run from the main loop.
;      Each state = event generator (EVENT_GEN_n) + action table (ACTIONS_n),
;      both dispatched with jump tables (JMP @A+DPTR).
;    - Timer 0 ISR : 50 ms time base -> tick100ms / tick1s / tick4s /
;                    tick30s / tick60s software flags.
;    - Version 1.1 : bug-fix release, see CHANGELOG.md
;    - ADC ISR     : end-of-conversion -> tick_adc0 / tick_adc1 flags.
;    - PWM0        : roller motor.  PWM1 : air dryer.
;
;  Distance annotation (kept from the original report)
;    "H" = horizontal distance from the VERTICAL rollers to the car  -> AIN0
;    "V" = vertical distance from the HORIZONTAL roller to the car   -> AIN1
;    ADC thresholds (50 mV/cm, 0-5 V, 8 bit): 0x4D=30cm 0x66=40cm 0x80=50cm
;
;  Naming: VR_ = vertical rollers, HR_ = horizontal roller, LS_ = limit switch,
;          S_ = sensor, ACT = action, EVENT = event code (register R7)
;==============================================================================

;------------------------------------------------------------------------------
; SYMBOL DEFINITIONS (EQU)
;------------------------------------------------------------------------------
LIGHT_GREEN     EQU	P0.0		;out: traffic light, green
LIGHT_RED       EQU	P0.1		;out: traffic light, red
S_PLATFORM      EQU	P0.2		;in : car on the entry platform
TOKEN_TYPE      EQU	P0.3		;in : token type (0 = normal, 1 = intense)
S_TOKEN         EQU	P0.4		;in : token inserted
BTN_START       EQU	P0.5		;in : START push-button
ALARM_START     EQU	P0.6		;out: "press START" alarm
S_BRIDGE_HOME   EQU	P0.7		;in : bridge at home position
BRIDGE_FWD      EQU	P1.0		;out: bridge moves forward
BRIDGE_BACK     EQU	P1.1		;out: bridge moves backward
VALVE_WATER     EQU	P2.0		;out: water solenoid valve
VALVE_SOAP      EQU	P2.1		;out: soap solenoid valve
S_CAR           EQU	P2.3		;in : car detected under the bridge
LS_VR_CENTER    EQU	P2.4		;in : vertical rollers limit switch, centre
LS_VR_EDGE      EQU	P2.5		;in : vertical rollers limit switch, edge
LS_HR_TOP       EQU	P1.2		;in : horizontal roller limit switch, top
LED_NORMAL      EQU	P2.6		;out: "normal program" LED
LED_INTENSE     EQU	P2.7		;out: "intense program" LED
VR_IN           EQU	P1.7		;out: vertical rollers move in
VR_OUT          EQU	P1.6		;out: vertical rollers move out
HR_DOWN         EQU	P1.4		;out: horizontal roller moves down
HR_UP           EQU	P1.3		;out: horizontal roller moves up
STATE           EQU	R6		;current FSM state (0..20)
EVENT           EQU	R7		;event code produced by EVENT_GEN_n
PWM0            EQU	0xFC		;PWM0 duty register - roller motor
PWM1            EQU	0xFD		;PWM1 duty register - air dryer
PWMP            EQU	0xFE		;PWM prescaler
IEN0            EQU	0xA8		;interrupt enable register (EA, EAD, ET0...)
ADCON           EQU	0xC5		;ADC control register
ADCH            EQU	0xC6		;ADC result (8 MSB)
cnt50ms         EQU	0x22		;50 ms counter (incremented by the Timer 0 ISR)
cnt1s           EQU	0x23		;seconds counter
tick100ms       EQU	0x20.0		;tick flags set by Flag_Compare
tick1s          EQU	0x20.1
tick30s         EQU	0x20.2
tick60s         EQU	0x20.3
tick4s          EQU	0x20.4
tick_adc0       EQU	0x20.5		;conversion finished on AIN0 (set by ADC ISR)
tick_adc1       EQU	0x20.6		;conversion finished on AIN1 (set by ADC ISR)
dist_ok         EQU	0x20.7		;E8: H distance reached
height_ok       EQU	0x21.1		;E8: V distance reached
VR_IN_ON        EQU	0x21.2		;E9: correction in progress, rollers IN
VR_OUT_ON       EQU	0x21.3		;E9: correction in progress, rollers OUT
HR_UP_ON        EQU	0x21.4		;E9: correction in progress, roller UP
HR_DOWN_ON      EQU	0x21.5		;E9: correction in progress, roller DOWN
bridge_reversed EQU	0x21.6		;E7: bridge already reversed at the roller edge

ORG	0x00
	AJMP	Main

;------------------------------------------------------------------------------
; TIMER 0 INTERRUPT (vector 0x0B) - 50 ms time base
;------------------------------------------------------------------------------

ORG	0x0B

	PUSH	ACC
	PUSH	PSW
	MOV	TL0,#0xB0
	MOV	TH0,#0x3C
	INC	cnt50ms
	ACALL	Unit_Conversion
	ACALL	Flag_Compare
	POP	PSW
	POP	ACC
	RETI

;------------------------------------------------------------------------------
; ADC INTERRUPT (vector 0x53) - end of conversion
;------------------------------------------------------------------------------
ORG	0x53

	PUSH	ACC
	PUSH	PSW
	ANL	ADCON,#0xEF		;clear ADCI (ADC interrupt flag)
	MOV	A,ADCON
	ANL	A,#0x07
	JZ	ADC0_Done
	SETB	tick_adc1
	AJMP	ADC_ISR_Exit

ADC0_Done:
	SETB	tick_adc0

ADC_ISR_Exit:
	POP	PSW
	POP	ACC
	RETI

ORG	0x7F
Main:
	ACALL	Init
Main_Loop:
	ACALL	State_Machine
	AJMP	Main_Loop

Init:
	MOV	STATE,#0x00		;initial state: E0 Idle
	MOV	EVENT,#0x00
	ANL	TMOD,#0xF0		;program Timer 0 (mode 1, 16-bit)
	ORL	TMOD,#0x01
	SETB	S_PLATFORM		;configure inputs (write 1 = input)
	SETB	TOKEN_TYPE
	SETB	S_TOKEN
	SETB	BTN_START
	SETB	S_CAR
	SETB	S_BRIDGE_HOME
	SETB	LS_VR_CENTER
	SETB	LS_VR_EDGE
	SETB	LS_HR_TOP
	SETB	LIGHT_GREEN		;configure outputs (initial values)
	CLR	LIGHT_RED
	CLR	ALARM_START
	CLR	LED_NORMAL
	CLR	LED_INTENSE
	CLR	VALVE_WATER
	CLR	VALVE_SOAP
	CLR	BRIDGE_BACK
	CLR	BRIDGE_FWD
	CLR	VR_IN
	CLR	VR_OUT
	CLR	HR_DOWN
	CLR	HR_UP
	MOV	PWMP,#0x17		;PWM prescaler: 12 MHz/(2*255*24) = 0.98 kHz
	MOV	PWM0,#0xFF
	MOV	PWM1,#0xFF
	MOV	cnt50ms,#0x00		;clear all counters and flags
	MOV	cnt1s,#0x00
	CLR	tick100ms
	CLR	tick1s
	CLR	tick30s
	CLR	tick60s
	CLR	tick4s
	CLR	tick_adc0
	CLR	tick_adc1
	CLR	bridge_reversed
	CLR	dist_ok
	CLR	height_ok
	CLR	VR_IN_ON
	CLR	VR_OUT_ON
	CLR	HR_UP_ON
	CLR	HR_DOWN_ON
	RET

;------------------------------------------------------------------------------
; STATE MACHINE DISPATCHER  (STATE in R6, jump table)
;------------------------------------------------------------------------------

State_Machine:
	MOV	A,STATE
	RL	A
	MOV	DPTR,#State_Table
	JMP	@A+DPTR
State_Table:
	AJMP	E0_Idle
	AJMP	E1_Wait_Token
	AJMP	E2_Wait_Start
	AJMP	E3_Approach
	AJMP	E4_Soap
	AJMP	E5_Position
	AJMP	E6_Wash_Rear
	AJMP	E7_Place_Rollers
	AJMP	E8_Approach_Rollers
	AJMP	E9_Wash
	AJMP	E10_Raise_HRoller
	AJMP	E11_Position
	AJMP	E12_Wash_Front
	AJMP	E13_Place_Rollers
	AJMP	E14_Position_Rinse
	AJMP	E15_Rinse
	AJMP	E16_Position_40cm
	AJMP	E17_Approach_Car
	AJMP	E18_Dry
	AJMP	E19_Return_Home
	AJMP	E20_Blink

;------------------------------------------------------------------------------
; STATES
;------------------------------------------------------------------------------
;------------------------------------------------------------------------------
; E0  IDLE            Wait for a car on the platform (S_PLATFORM).
;------------------------------------------------------------------------------

E0_Idle:

	ACALL	EVENT_GEN_0
	MOV	A,EVENT
	RL	A
	MOV	DPTR,#ACTIONS_0
	JMP	@A+DPTR
ACTIONS_0:
	AJMP	E0_ACT0
	AJMP	E0_ACT1

EVENT_GEN_0:
	JB	S_PLATFORM,EVENT0_1
;EVENT0_0:			;no event (default)
	MOV	EVENT,#0x00
	RET
EVENT0_1:
	MOV	EVENT,#0x01
	RET

E0_ACT0:
	RET

E0_ACT1:
	SETB	LIGHT_RED
	CLR	LIGHT_GREEN
	MOV	STATE,#0x01
	ACALL	Timer_Start
	RET

;------------------------------------------------------------------------------
; E1  WAIT_TOKEN      Wait for a token. 30 s -> alarm. Car leaves -> back to E0.
;------------------------------------------------------------------------------

E1_Wait_Token:
	ACALL	EVENT_GEN_1
	MOV	A,EVENT
	RL	A
	MOV	DPTR,#ACTIONS_1
	JMP	@A+DPTR
ACTIONS_1:
	AJMP	E1_ACT0
	AJMP	E1_ACT1
	AJMP	E1_ACT2
	AJMP	E1_ACT3

EVENT_GEN_1:
	JNB	S_PLATFORM,EVENT1_3
	JB	tick30s,EVENT1_2
	JB	S_TOKEN,EVENT1_1
;EVENT1_0:			;no event (default)
	MOV	EVENT,#0x00
	RET
EVENT1_1:
	MOV	EVENT,#0x01
	RET
EVENT1_2:
	CLR	tick30s
	MOV	EVENT,#0x02
	RET
EVENT1_3:
	MOV	EVENT,#0x03
	RET

E1_ACT0:
	RET

E1_ACT1:
	CLR	ALARM_START
	CLR	tick30s
	CLR	tick60s
	MOV	cnt50ms,#0x00		;restart the 30 s / 60 s count
	MOV	cnt1s,#0x00
	MOV	STATE,#0x02
	RET

E1_ACT2:
	SETB	ALARM_START
	RET

E1_ACT3:
	SETB	LIGHT_GREEN
	CLR	LIGHT_RED
	CLR	ALARM_START
	ACALL	Timer_Stop
	MOV	STATE,#0x00
	RET

;------------------------------------------------------------------------------
; E2  WAIT_START      Program LED by token type. START button or 60 s -> go. 30 s -> alarm.
;------------------------------------------------------------------------------

E2_Wait_Start:
	ACALL	EVENT_GEN_2
	MOV	A,EVENT
	RL	A
	MOV	DPTR,#ACTIONS_2
	JMP	@A+DPTR
ACTIONS_2:
	AJMP	E2_ACT0
	AJMP	E2_ACT1
	AJMP	E2_ACT2
	AJMP	E2_ACT3

EVENT_GEN_2:
	JB	tick60s,EVENT2_1
	JB	tick30s,EVENT2_0
	JB	BTN_START,EVENT2_1
	JNB	TOKEN_TYPE,EVENT2_2
	JB	TOKEN_TYPE,EVENT2_3
EVENT2_0:
	CLR	tick30s
	MOV	EVENT,#0x00
	RET
EVENT2_1:
	CLR	tick60s
	MOV	EVENT,#0x01
	RET
EVENT2_2:
	MOV	EVENT,#0x02
	RET
EVENT2_3:
	MOV	EVENT,#0x03
	RET

E2_ACT0:
	SETB	ALARM_START
	RET

E2_ACT1:
	SETB	BRIDGE_FWD
	CLR	ALARM_START
	ACALL	Timer_Stop
	MOV	STATE,#0x03
	RET

E2_ACT2:
	SETB	LED_NORMAL
	RET

E2_ACT3:
	SETB	LED_INTENSE
	RET

;------------------------------------------------------------------------------
; E3  APPROACH        Bridge moves forward until the car is detected -> water + soap ON.
;------------------------------------------------------------------------------

E3_Approach:
	ACALL	EVENT_GEN_3
	MOV	A,EVENT
	RL	A
	MOV	DPTR,#ACTIONS_3
	JMP	@A+DPTR
ACTIONS_3:
	AJMP	E3_ACT0
	AJMP	E3_ACT1
EVENT_GEN_3:
	JB	S_CAR,EVENT3_1
;EVENT3_0:			;no event (default)
	MOV	EVENT,#0x00
	RET
EVENT3_1:
	MOV	EVENT,#0x01
	RET

E3_ACT0:
	RET

E3_ACT1:
	SETB	VALVE_WATER
	SETB	VALVE_SOAP
	MOV	STATE,#0x04
	RET

;------------------------------------------------------------------------------
; E4  SOAP            Pre-wash until the car is no longer detected -> water + soap OFF.
;------------------------------------------------------------------------------

E4_Soap:
	ACALL	EVENT_GEN_4
	MOV	A,EVENT
	RL	A
	MOV	DPTR,#ACTIONS_4
	JMP	@A+DPTR
ACTIONS_4:
	AJMP	E4_ACT0
	AJMP	E4_ACT1
EVENT_GEN_4:
	JNB	S_CAR,EVENT4_1
;EVENT4_0:			;no event (default)
	MOV	EVENT,#0x00
	RET
EVENT4_1:
	MOV	EVENT,#0x01
	RET

E4_ACT0:
	RET

E4_ACT1:
	CLR	VALVE_WATER
	CLR	VALVE_SOAP
	ACALL	Timer_Start
	MOV	STATE,#0x05
	RET

;------------------------------------------------------------------------------
; E5  POSITION        Bridge keeps moving 4 s -> stop, roller motor ON (PWM0), rollers IN.
;------------------------------------------------------------------------------

E5_Position:
	ACALL	EVENT_GEN_5
	MOV	A,EVENT
	RL	A
	MOV	DPTR,#ACTIONS_5
	JMP	@A+DPTR
ACTIONS_5:
	AJMP	E5_ACT0
	AJMP	E5_ACT1

EVENT_GEN_5:
	JB	tick4s,EVENT5_1
;EVENT5_0:			;no event (default)
	MOV	EVENT,#0x00
	RET
EVENT5_1:
	CLR	tick4s
	MOV	EVENT,#0x01
	RET

E5_ACT0:
	RET

E5_ACT1:
	ACALL	PWM0_On
	CLR	BRIDGE_FWD
	SETB	VR_IN
	ACALL	Timer_Stop
	MOV	STATE,#0x06
	RET

;------------------------------------------------------------------------------
; E6  WASH_REAR       Vertical rollers close in until the centre limit switch.
;------------------------------------------------------------------------------

E6_Wash_Rear:
	ACALL	EVENT_GEN_6
	MOV	A,EVENT
	RL	A
	MOV	DPTR,#ACTIONS_6
	JMP	@A+DPTR
ACTIONS_6:
	AJMP	E6_ACT0
	AJMP	E6_ACT1

EVENT_GEN_6:
	JB	LS_VR_CENTER,EVENT6_1
;EVENT6_0:			;no event (default)
	MOV	EVENT,#0x00
	RET
EVENT6_1:
	MOV	EVENT,#0x01
	RET

E6_ACT0:
	RET

E6_ACT1:
	CLR	VR_IN
	SETB	VR_OUT
	MOV	STATE,#0x07
	RET

;------------------------------------------------------------------------------
; E7  PLACE_ROLLERS   Rollers OUT to edge, bridge back until car -> rollers IN, H-roller DOWN.
;------------------------------------------------------------------------------

E7_Place_Rollers:
	ACALL	EVENT_GEN_7
	MOV	A,EVENT
	RL	A
	MOV	DPTR,#ACTIONS_7
	JMP	@A+DPTR
ACTIONS_7:
	AJMP	E7_ACT0
	AJMP	E7_ACT1
	AJMP	E7_ACT2

EVENT_GEN_7:
	JB	S_CAR,EVENT7_2
	JB	LS_VR_EDGE,Check_Flag
EVENT7_0:
	MOV	EVENT,#0x00
	RET
EVENT7_1:
	MOV	EVENT,#0x01
	RET
EVENT7_2:
	MOV	EVENT,#0x02
	RET
Check_Flag:
	JB	bridge_reversed,EVENT7_0
	JMP	EVENT7_1

E7_ACT0:
	RET

E7_ACT1:
	CLR	VR_OUT
	SETB	BRIDGE_BACK
	SETB	bridge_reversed
	RET

E7_ACT2:
	SETB	VR_IN
	SETB	HR_DOWN
	CLR	BRIDGE_BACK
	CLR	bridge_reversed		;ready for the next car
	CLR	dist_ok			;E8 starts with both distances unknown
	CLR	height_ok
	MOV	STATE,#0x08
	ACALL	Timer_Start
	RET

;------------------------------------------------------------------------------
; E8  APPROACH_ROLL.  Bring both rollers to ~40 cm of the car using ADC0 / ADC1.
;------------------------------------------------------------------------------

E8_Approach_Rollers:
	ACALL	EVENT_GEN_8
	MOV	A,EVENT
	RL	A
	MOV	DPTR,#ACTIONS_8
	JMP	@A+DPTR
ACTIONS_8:
	AJMP	E8_ACT0
	AJMP	E8_ACT1
	AJMP	E8_ACT2
	AJMP	E8_ACT3
	AJMP	E8_ACT4
	AJMP	E8_ACT5
	AJMP	E8_ACT6

EVENT_GEN_8:
	JB	tick100ms,EVENT8_TICK
	JB	tick_adc0,EVENT8_ADC0
	JB	tick_adc1,EVENT8_ADC1
;EVENT8_0:			;no event (default)
	MOV	EVENT,#0x00
	RET
EVENT8_TICK:
	CLR	tick100ms
	JB	dist_ok,H_Already_OK
	MOV	EVENT,#0x01		;request ADC0
	RET
H_Already_OK:
	MOV	EVENT,#0x02		;H is already OK, request ADC1
	RET
EVENT8_ADC0:				;compare ADC0
	CLR	tick_adc0
	MOV	A,ADCH
	CLR	C
	CJNE	A,#0x69,H_NOK
H_OK:					;ADC0 OK, check whether ADC1 is already OK
	SETB	dist_ok
	JB	height_ok,V_OK_H_OK
	MOV	EVENT,#0x03		;V not OK yet, H OK
	RET
V_OK_H_OK:				;both OK
	MOV	EVENT,#0x04
	RET
H_NOK:					;ADC0 not OK, check whether ADC1 is already OK
	JC	H_OK
	JB	height_ok,V_OK_H_NOK
	MOV	EVENT,#0x02		;both not OK
	RET
V_OK_H_NOK:
	MOV	EVENT,#0x00		;V OK
	RET
EVENT8_ADC1:				;compare V (ADC1)
	CLR	tick_adc1
	MOV	A,ADCH
	CLR	C
	CJNE	A,#0x69,V_NOK
V_OK:					;V OK
	SETB	height_ok
	JB	dist_ok,H_OK_V_OK
	MOV	EVENT,#0x05		;H still not OK
	RET
H_OK_V_OK:				;both OK
	MOV	EVENT,#0x06
	RET
V_NOK:
	JC	V_OK			;V not OK, wait for the next 100 ms tick
	MOV	EVENT,#0x00
	RET

E8_ACT0:
	RET

E8_ACT1:
	MOV	A,#0x00
	ACALL	ADC_Start
	RET

E8_ACT2:
	MOV	A,#0x01
	ACALL	ADC_Start
	RET

E8_ACT3:
	CLR	VR_IN
	MOV	A,#0x01
	ACALL	ADC_Start
	RET

E8_ACT4:
	CLR	VR_IN
	SETB	BRIDGE_BACK
	MOV	STATE,#0x09
	RET

E8_ACT5:
	CLR	HR_DOWN
	RET

E8_ACT6:
	CLR	HR_DOWN
	SETB	BRIDGE_BACK
	MOV	STATE,#0x09
	RET

;------------------------------------------------------------------------------
; E9  WASH            Closed-loop 30..50 cm distance tracking while the bridge moves back.
;------------------------------------------------------------------------------

E9_Wash:
	ACALL	EVENT_GEN_9
	MOV	A,EVENT
	RL	A
	MOV	DPTR,#ACTIONS_9
	JMP	@A+DPTR
ACTIONS_9:
	AJMP	E9_ACT0
	AJMP	E9_ACT1
	AJMP	E9_ACT2
	AJMP	E9_ACT3
	AJMP	E9_ACT4
	AJMP	E9_ACT5
	AJMP	E9_ACT6
	AJMP	E9_ACT7
	AJMP	E9_ACT8
	AJMP	E9_ACT9
	AJMP	E9_ACT10
	AJMP	E9_ACT11

EVENT_GEN_9:
	JNB	S_CAR,EVENT9_9
	JB	tick100ms,EVENT9_1
	JB	tick_adc0,EVENT9_ADC0
	JB	tick_adc1,EVENT9_ADC1
;EVENT9_0:			;no event (default)
	MOV	EVENT,#0x00
	RET
EVENT9_1:
	CLR	tick100ms
	MOV	EVENT,#0x01
	RET
EVENT9_9:
	MOV	EVENT,#0x09
	RET
EVENT9_ADC0:
	JB	VR_IN_ON,vr_in_is_on
	JB	VR_OUT_ON,vr_out_is_on
	CLR	tick_adc0
	MOV	A,ADCH
	CLR	C
	CJNE	A,#0x4D,H_LT_30
H_NOT_LT_30:
	CLR	C
	CJNE	A,#0x80,H_GT_50
	AJMP	H_NOT_GT_50
H_LT_30:
	JNC	H_NOT_LT_30
	SETB	VR_OUT_ON
	MOV	EVENT,#0x02
	RET
H_GT_50:
	JC	H_NOT_GT_50
	SETB	VR_IN_ON
	MOV	EVENT,#0x03
	RET
H_NOT_GT_50:
	MOV	EVENT,#0x0B		;H in range: go on with V
	RET
vr_in_is_on:
	CLR	tick_adc0
	MOV	A,ADCH
	CLR	C
	CJNE	A,#0x66,H_GT_40
H_LT_40:
	CLR	VR_IN_ON
	MOV	EVENT,#0x05
	RET
H_GT_40:
	JC	H_LT_40
	MOV	EVENT,#0x0B		;still moving: go on with V
	RET
vr_out_is_on:
	CLR	tick_adc0
	MOV	A,ADCH
	CLR	C
	CJNE	A,#0x66,H_LT_40_2
H_GT_40_2:
	CLR	VR_OUT_ON
	MOV	EVENT,#0x04
	RET
H_LT_40_2:
	JNC	H_GT_40_2
	MOV	EVENT,#0x0B		;still moving: go on with V
	RET
EVENT9_ADC1:
	JB	HR_UP_ON,hr_up_is_on
	JB	HR_DOWN_ON,hr_down_is_on
	CLR	tick_adc1
	MOV	A,ADCH
	CLR	C
	CJNE	A,#0x4D,V_LT_30
V_NOT_LT_30:
	CLR	C
	CJNE	A,#0x80,V_GT_50
	AJMP	V_NOT_GT_50
V_LT_30:
	JNC	V_NOT_LT_30
	SETB	HR_UP_ON
	MOV	EVENT,#0x06
	RET
V_GT_50:
	JC	V_NOT_GT_50
	SETB	HR_DOWN_ON
	MOV	EVENT,#0x07
	RET
V_NOT_GT_50:
	MOV	EVENT,#0x00
	RET
hr_up_is_on:
	CLR	tick_adc1
	MOV	A,ADCH
	CLR	C
	CJNE	A,#0x66,V_LT_40
V_GT_40:
	CLR	HR_UP_ON
	MOV	EVENT,#0x0A
	RET
V_LT_40:
	JNC	V_GT_40
	MOV	EVENT,#0x00
	RET
hr_down_is_on:
	CLR	tick_adc1
	MOV	A,ADCH
	CLR	C
	CJNE	A,#0x66,V_GT_40_2
V_LT_40_2:
	CLR	HR_DOWN_ON
	MOV	EVENT,#0x08
	RET
V_GT_40_2:
	JC	V_LT_40_2
	MOV	EVENT,#0x00
	RET

E9_ACT0:
	RET

E9_ACT1:
	MOV	A,#0x00
	ACALL	ADC_Start
	RET

E9_ACT2:
	SETB	VR_OUT
	MOV	A,#0x01
	ACALL	ADC_Start
	RET

E9_ACT3:
	SETB	VR_IN
	MOV	A,#0x01
	ACALL	ADC_Start
	RET

E9_ACT4:
	CLR	VR_OUT
	MOV	A,#0x01
	ACALL	ADC_Start
	RET

E9_ACT5:
	CLR	VR_IN
	MOV	A,#0x01
	ACALL	ADC_Start
	RET

E9_ACT6:
	SETB	HR_UP
	RET

E9_ACT7:
	SETB	HR_DOWN
	RET

E9_ACT8:
	CLR	HR_DOWN
	RET

E9_ACT9:
	CLR	BRIDGE_BACK
	CLR	VR_IN
	CLR	HR_DOWN
	CLR	VR_OUT
	SETB	HR_UP
	CLR	VR_IN_ON		;no correction in progress any more
	CLR	VR_OUT_ON
	CLR	HR_UP_ON
	CLR	HR_DOWN_ON
	ACALL	Timer_Stop
	MOV	STATE,#0x0A
	RET

E9_ACT10:
	CLR	HR_UP
	RET

E9_ACT11:
	MOV	A,#0x01			;request ADC1 (V)
	ACALL	ADC_Start
	RET

;------------------------------------------------------------------------------
; E10 RAISE_HROLLER   Raise the horizontal roller until its top limit switch.
;------------------------------------------------------------------------------

E10_Raise_HRoller:
	ACALL	EVENT_GEN_10
	MOV	A,EVENT
	RL	A
	MOV	DPTR,#ACTIONS_10
	JMP	@A+DPTR
ACTIONS_10:
	AJMP	E10_ACT0
	AJMP	E10_ACT1

EVENT_GEN_10:
	JB	LS_HR_TOP,EVENT10_1
;EVENT10_0:			;no event (default)
	MOV	EVENT,#0x00
	RET
EVENT10_1:
	MOV	EVENT,#0x01
	RET

E10_ACT0:
	RET

E10_ACT1:
	SETB	BRIDGE_BACK
	CLR	HR_UP
	ACALL	Timer_Start
	MOV	STATE,#0x0B
	RET

;------------------------------------------------------------------------------
; E11 POSITION        Bridge moves back 4 s -> stop, vertical rollers IN.
;------------------------------------------------------------------------------

E11_Position:
	ACALL	EVENT_GEN_11
	MOV	A,EVENT
	RL	A
	MOV	DPTR,#ACTIONS_11
	JMP	@A+DPTR
ACTIONS_11:
	AJMP	E11_ACT0
	AJMP	E11_ACT1

EVENT_GEN_11:
	JB	tick4s,EVENT11_1
;EVENT11_0:			;no event (default)
	MOV	EVENT,#0x00
	RET
EVENT11_1:
	MOV	EVENT,#0x01
	RET

E11_ACT0:
	RET

E11_ACT1:
	CLR	BRIDGE_BACK
	SETB	VR_IN
	ACALL	Timer_Stop
	MOV	STATE,#0x0C
	RET

;------------------------------------------------------------------------------
; E12 WASH_FRONT      Vertical rollers close in until the centre limit switch.
;------------------------------------------------------------------------------

E12_Wash_Front:
	ACALL	EVENT_GEN_12
	MOV	A,EVENT
	RL	A
	MOV	DPTR,#ACTIONS_12
	JMP	@A+DPTR
ACTIONS_12:
	AJMP	E12_ACT0
	AJMP	E12_ACT1

EVENT_GEN_12:
	JB	LS_VR_CENTER,EVENT12_1
;EVENT12_0:			;no event (default)
	MOV	EVENT,#0x00
	RET
EVENT12_1:
	MOV	EVENT,#0x01
	RET

E12_ACT0:
	RET

E12_ACT1:
	CLR	VR_IN
	SETB	VR_OUT
	MOV	STATE,#0x0D
	RET

;------------------------------------------------------------------------------
; E13 PLACE_ROLLERS   Rollers OUT to edge -> bridge forward, roller motor OFF.
;------------------------------------------------------------------------------

E13_Place_Rollers:
	ACALL	EVENT_GEN_13
	MOV	A,EVENT
	RL	A
	MOV	DPTR,#ACTIONS_13
	JMP	@A+DPTR
ACTIONS_13:
	AJMP	E13_ACT0
	AJMP	E13_ACT1

EVENT_GEN_13:
	JB	LS_VR_EDGE,EVENT13_1
;EVENT13_0:			;no event (default)
	MOV	EVENT,#0x00
	RET
EVENT13_1:
	MOV	EVENT,#0x01
	RET

E13_ACT0:
	RET

E13_ACT1:
	CLR	VR_OUT			;rollers reached the edge
	SETB	BRIDGE_FWD
	ACALL	PWM0_Off
	MOV	STATE,#0x0E
	RET

;------------------------------------------------------------------------------
; E14 POSITION_RINSE  Bridge forward until the car is detected -> water ON.
;------------------------------------------------------------------------------

E14_Position_Rinse:
	ACALL	EVENT_GEN_14
	MOV	A,EVENT
	RL	A
	MOV	DPTR,#ACTIONS_14
	JMP	@A+DPTR
ACTIONS_14:
	AJMP	E14_ACT0
	AJMP	E14_ACT1

EVENT_GEN_14:
	JB	S_CAR,EVENT14_1
;EVENT14_0:			;no event (default)
	MOV	EVENT,#0x00
	RET
EVENT14_1:
	MOV	EVENT,#0x01
	RET

E14_ACT0:
	RET

E14_ACT1:
	SETB	VALVE_WATER
	MOV	STATE,#0x0F
	RET

;------------------------------------------------------------------------------
; E15 RINSE           Rinse until the car is no longer detected -> water OFF.
;------------------------------------------------------------------------------

E15_Rinse:
	ACALL	EVENT_GEN_15
	MOV	A,EVENT
	RL	A
	MOV	DPTR,#ACTIONS_15
	JMP	@A+DPTR
ACTIONS_15:
	AJMP	E15_ACT0
	AJMP	E15_ACT1

EVENT_GEN_15:
	JNB	S_CAR,EVENT15_1
;EVENT15_0:			;no event (default)
	MOV	EVENT,#0x00
	RET
EVENT15_1:
	MOV	EVENT,#0x01
	RET

E15_ACT0:
	RET

E15_ACT1:
	CLR	VALVE_WATER
	ACALL	Timer_Start
	MOV	STATE,#0x10
	RET

;------------------------------------------------------------------------------
; E16 POSITION_40CM   Bridge keeps moving 4 s -> reverse direction.
;------------------------------------------------------------------------------

E16_Position_40cm:
	ACALL	EVENT_GEN_16
	MOV	A,EVENT
	RL	A
	MOV	DPTR,#ACTIONS_16
	JMP	@A+DPTR
ACTIONS_16:
	AJMP	E16_ACT0
	AJMP	E16_ACT1

EVENT_GEN_16:
	JB	tick4s,EVENT16_1
;EVENT16_0:			;no event (default)
	MOV	EVENT,#0x00
	RET
EVENT16_1:
	CLR	tick4s
	MOV	EVENT,#0x01
	RET

E16_ACT0:
	RET

E16_ACT1:
	CLR	BRIDGE_FWD
	SETB	BRIDGE_BACK
	ACALL	Timer_Stop
	MOV	STATE,#0x11
	RET

;------------------------------------------------------------------------------
; E17 APPROACH_CAR    Bridge back until the car is detected -> dryer ON (PWM1).
;------------------------------------------------------------------------------

E17_Approach_Car:
	ACALL	EVENT_GEN_17
	MOV	A,EVENT
	RL	A
	MOV	DPTR,#ACTIONS_17
	JMP	@A+DPTR
ACTIONS_17:
	AJMP	E17_ACT0
	AJMP	E17_ACT1

EVENT_GEN_17:
	JB	S_CAR,EVENT17_1
;EVENT17_0:			;no event (default)
	MOV	EVENT,#0x00
	RET
EVENT17_1:
	MOV	EVENT,#0x01
	RET

E17_ACT0:
	RET

E17_ACT1:
	ACALL	PWM1_On
	MOV	STATE,#0x12
	RET

;------------------------------------------------------------------------------
; E18 DRY             Dry until the car is no longer detected -> dryer OFF.
;------------------------------------------------------------------------------

E18_Dry:
	ACALL	EVENT_GEN_18
	MOV	A,EVENT
	RL	A
	MOV	DPTR,#ACTIONS_18
	JMP	@A+DPTR
ACTIONS_18:
	AJMP	E18_ACT0
	AJMP	E18_ACT1

EVENT_GEN_18:
	JNB	S_CAR,EVENT18_1
;EVENT18_0:			;no event (default)
	MOV	EVENT,#0x00
	RET
EVENT18_1:
	MOV	EVENT,#0x01
	RET

E18_ACT0:
	RET

E18_ACT1:
	ACALL	PWM1_Off
	MOV	STATE,#0x13
	RET

;------------------------------------------------------------------------------
; E19 RETURN_HOME     Bridge back to home sensor -> stop, LEDs off, green light.
;------------------------------------------------------------------------------

E19_Return_Home:
	ACALL	EVENT_GEN_19
	MOV	A,EVENT
	RL	A
	MOV	DPTR,#ACTIONS_19
	JMP	@A+DPTR
ACTIONS_19:
	AJMP	E19_ACT0
	AJMP	E19_ACT1

EVENT_GEN_19:
	JB	S_BRIDGE_HOME,EVENT19_1
;EVENT19_0:			;no event (default)
	MOV	EVENT,#0x00
	RET
EVENT19_1:
	MOV	EVENT,#0x01
	RET

E19_ACT0:
	RET

E19_ACT1:
	ACALL	Timer_Start
	CLR	BRIDGE_BACK
	CLR	LED_INTENSE
	CLR	LED_NORMAL
	CLR	LIGHT_RED
	SETB	LIGHT_GREEN
	MOV	STATE,#0x14
	RET

;------------------------------------------------------------------------------
; E20 BLINK           Green light blinks every 1 s until the car leaves -> E0.
;------------------------------------------------------------------------------

E20_Blink:
	ACALL	EVENT_GEN_20
	MOV	A,EVENT
	RL	A
	MOV	DPTR,#ACTIONS_20
	JMP	@A+DPTR
ACTIONS_20:
	AJMP	E20_ACT0
	AJMP	E20_ACT1
	AJMP	E20_ACT2

EVENT_GEN_20:
	JB	tick1s,EVENT20_1
	JNB	S_PLATFORM,EVENT20_2
;EVENT20_0:			;no event (default)
	MOV	EVENT,#0x00
	RET
EVENT20_1:
	CLR	tick1s
	MOV	EVENT,#0x01
	RET
EVENT20_2:
	MOV	EVENT,#0x02
	RET

E20_ACT0:
	RET

E20_ACT1:
	ACALL	Timer_Restart
	CPL	LIGHT_GREEN
	RET

E20_ACT2:
	SETB	LIGHT_GREEN
	CLR	LIGHT_RED
	ACALL	Timer_Stop
	MOV	STATE,#0x00
	RET

;------------------------------------------------------------------------------
; TIMER 0 HELPERS: start / stop / restart
;------------------------------------------------------------------------------

Timer_Start:
	ACALL	Timer_Restart		;reload and clear counters / ticks
	SETB	TR0
	RET

;------------------------------------------------------------------------------
; Timer_Stop (stop Timer 0) / Timer_Restart (reload, keep running): clear counters and ticks
;------------------------------------------------------------------------------

Timer_Stop:
	CLR	TR0
	MOV	cnt50ms,#0x00
	MOV	cnt1s,#0x00
	CLR	tick100ms
	CLR	tick1s
	CLR	tick30s
	CLR	tick60s
	CLR	tick4s
	RET

Timer_Restart:
	MOV	TL0,#0xB0
	MOV	TH0,#0x3C
	ORL	IEN0,#0x82
	MOV	cnt50ms,#0x00
	MOV	cnt1s,#0x00
	CLR	tick100ms
	CLR	tick1s
	CLR	tick30s
	CLR	tick60s
	CLR	tick4s
	RET

;------------------------------------------------------------------------------
; TIMER COUNTERS AND TICK FLAGS (called from the Timer 0 ISR, every 50 ms)
;   cnt50ms counts 0..19 and wraps every second, cnt1s counts seconds.
;   tick100ms : every second interrupt (cnt50ms even)
;   tick1s    : every time cnt50ms wraps
;   tick4s / tick30s / tick60s : once, when cnt1s reaches 4 / 30 / 60
;------------------------------------------------------------------------------

Unit_Conversion:
	MOV	A,cnt50ms
	CLR	C
	SUBB	A,#0x14			;20 x 50 ms = 1 s
	JC	end_UC			;cnt50ms < 20
	MOV	cnt50ms,#0x00
	INC	cnt1s
end_UC:
	RET

Flag_Compare:
	MOV	A,cnt50ms
	JB	ACC.0,Flags_Done	;odd count: no new 100 ms period
	SETB	tick100ms
	JNZ	Flags_Done		;not a full second yet
	SETB	tick1s
	MOV	A,cnt1s
	CJNE	A,#0x04,check30s
	SETB	tick4s
check30s:
	CJNE	A,#0x1E,check60s
	SETB	tick30s
check60s:
	CJNE	A,#0x3C,Flags_Done
	SETB	tick60s
Flags_Done:
	RET

;------------------------------------------------------------------------------
; ADC_Start: start a conversion on the channel given in A
;------------------------------------------------------------------------------

ADC_Start:
;	CLR	tick_adc0
;	CLR	tick_adc1
	ANL	ADCON,#0xC8
	ORL	ADCON,A
	ORL	IEN0,#0xC0
	ORL	ADCON,#0x08
	RET

;------------------------------------------------------------------------------
; SUBROUTINES: PWM control
;------------------------------------------------------------------------------
			;--- enable ---

PWM0_On:
	JB	TOKEN_TYPE,PWM0_Intense
	MOV	PWM0,#0x80		;50 %
	RET
PWM0_Intense:
	MOV	PWM0,#0x00		;100 %
	RET

PWM1_On:
	JB	TOKEN_TYPE,PWM1_Intense
	MOV	PWM1,#0x80		;50 %
	RET
PWM1_Intense:
	MOV	PWM1,#0x26		;85 %
	RET

			;--- disable ---

PWM0_Off:
	MOV	PWM0,#0xFF
	RET

PWM1_Off:
	MOV	PWM1,#0xFF
	RET

END
