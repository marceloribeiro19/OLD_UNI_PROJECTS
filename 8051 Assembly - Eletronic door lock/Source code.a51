#include <REG51F380.H>

K_SET EQU P0.6
K_LOAD EQU P0.7
S_RECOVER EQU 0
S_LOCKED EQU 1
S_DECRYPT EQU 2
S_OPEN EQU 3
S_ENCRYPT EQU 4
S_FAIL EQU 5
S_BLOCKED EQU 6
	
DSEG AT 30H
	STATE: DS 1
	NEXT_STATE: DS 1
	COUNT: DS 1      //UTILIZADA NO CICLO WHILE PARA COLOCAR OS 4 DIGITOS DA CHAVE
	INDEX: DS 1	
	TENTATIVAS_INVALIDAS: DS 1
ISEG AT 80H
	DIGIT_1: DS 1

CSEG AT 0H
	JMP INIT
CSEG AT 2BH
	CPL P1.0
RETI
CSEG AT 0F0H
	INIT:
		MOV P1MDOUT,#0FFH ; push pull
		MOV FLSCL, #90H ;48mhz
		MOV CLKSEL, #3
		MOV PCA0MD, #0 ;DESLIGAR WDT
		MOV XBR1, #40H
		MOV STATE,#S_RECOVER
		MOV NEXT_STATE,#S_RECOVER
		MOV TENTATIVAS_INVALIDAS,#0H

;-------MAQUINA-DE-ESTADOS-----------------;
STATE_MACHINE:
	MOV STATE, NEXT_STATE
	CALL STATE_JMP
	JNB K_SET, $
	JNB K_LOAD, $
	JMP STATE_MACHINE
	
STATE_JMP:
	MOV A, STATE
	RL A
	MOV DPTR, #TABLE_STATE_JUMP
	JMP @A+DPTR	
TABLE_STATE_JUMP:
	AJMP ROT_RECOVER
	AJMP ROT_LOCKED
	AJMP ROT_DECRYPT
	AJMP ROT_OPEN
	AJMP ROT_ENCRYPT
	AJMP ROT_FAIL
	AJMP ROT_BLOCKED
;----------ESTADO-LOCKED------------------------------------------;
ROT_LOCKED:		
MOV P4,#1						;tensao de bloqueio ativada
MOV P2,#0C7H					;apresenta o caracter 'L' no display
JB K_SET,$
JNB K_SET,$
CALL SUB_ROT_4DIGITS			;rotina para colocar escolher 4 digitos atraves dos botoes e do display
NEXT_STATE_LOCKED_DECRYPT:		;no fim da escolha o único estado possível é o estado decrypt
MOV NEXT_STATE, #S_DECRYPT
RET
;---------------ESTADO-OPEN-------------------;
ROT_OPEN:
MOV P2,#0C0H					;apresenta no display o caractér 'O'
MOV TENTATIVAS_INVALIDAS,#0H	;limpa o numero de tentativas de falha de abertura
MOV P4,#0	 					;tensão de bloqueio a zero
MOV R2,#30						;30 * 500ms = 15s
BLINK_DISPLAY:
	MOV P2,#040H				;apresenta o caracter '0' com o ponto ligado
	JNB K_LOAD,NEXT_STATE_OPEN_LOCKED
	ACALL ROT_DELAY500
	MOV P2,#0C0H				;apresenta o caracter '0' com o ponto desligado
	ACALL ROT_DELAY500
	JNB K_LOAD,NEXT_STATE_OPEN_LOCKED
	DJNZ R2,BLINK_DISPLAY
	AJMP NEXT_STATE_OPEN_ENCRYPT

	NEXT_STATE_OPEN_LOCKED:
		JNB K_LOAD,$
		MOV NEXT_STATE,#S_LOCKED
		RET	
	NEXT_STATE_OPEN_ENCRYPT:
		MOV NEXT_STATE,#S_ENCRYPT
		RET
		
;-------------Estado-Encrypt-------------------------------;
ROT_ENCRYPT:
CALL SUB_ROT_4DIGITS
MOV R0,#DIGIT_1
MOV R2,#4
ENCRYPT_DIGITS:
	MOV A,@R0
	MOV @R0,#0H					;limpar o byte que guarda o digito
	XRL A,#123					;xor com 1234h para obter o digito da passe encriptada
	MOVX @R0,A					;mover para a memória externa o digito
	INC R0						;incrementar r0 para este apontar para o proximo digito
	DJNZ R2,ENCRYPT_DIGITS		;decrementar r4 de 4 até zero para transformar  e guardar os 4 digitos
NEXT_STATE_ENCRYPT_LOCKED:
	MOV NEXT_STATE,#S_LOCKED
	RET
	
	
;-----------Estado-Decrypt----------------------
ROT_DECRYPT:
	MOV R0,#DIGIT_1             ;coloca em r0 a posiçao do primeiro digito
	MOV R2,#4					;ciclo vai fazer-se 4x
DECRYPT_DIGITS:				
	MOVX A,@R0					;coloca o digito encriptado no acumulador
	XRL A,#123				;desencripta o digito
	CLR C						;limpar carry para fazer a subtraçao
	SUBB A,@R0					;verifica de o digito da chave é igual ao selecionado pelo utilizador
	MOV @R0,#0					;limpa o digito selecionado pelo utilizador de modo a nao ficar a chave explicita na memoria
	JNZ NEXT_STATE_DECRYPT_FAIL	;se um digito nao for igual entao o proximo estado é o estado fail
	INC R0
	DJNZ R2,DECRYPT_DIGITS		;se o digito for igual, verifica os proximos até 4 digitos
NEXT_STATE_DECRYPT_OPEN:
	MOV NEXT_STATE,#S_OPEN
	RET
NEXT_STATE_DECRYPT_FAIL:
	MOV NEXT_STATE,#S_FAIL
	RET
	
	
;--------Estado-Fail------------------;
ROT_FAIL:
	MOV P2,#08EH					;apresenta o caracter 'F' no display
	INC TENTATIVAS_INVALIDAS		;numero de vezes que falhou a chave incrementa
	MOV A,TENTATIVAS_INVALIDAS
	CLR C
	SUBB A,#6						;verifica se o numero de tentativas falhadas ja ultrapassou os 5
	JZ NEXT_STATE_FAIL_BLOCKED		;se sim o proximo estado é o de bloqueio
	MOV R2,TENTATIVAS_INVALIDAS	
	WAIT_TIME:
		CALL ROT_DELAY_10S				;espera de 10 segundos até a proxima tentativa
		DJNZ R2,WAIT_TIME				;10 segundos por cada tentativa falhada	
	NEXT_STATE_FAIL_DECRYPT:
		CALL SUB_ROT_4DIGITS			;escolha de 4 novos digitos
		MOV NEXT_STATE,#S_DECRYPT		;proximo estado é o decrypt
		RET
	NEXT_STATE_FAIL_BLOCKED:
		MOV NEXT_STATE, #S_BLOCKED				;depois das 5 tentativas invalidas a fechadura bloqueia
		RET
		
;----- ROT_BLOCKED-------------
ROT_BLOCKED:											
		MOV TMR2L,#LOW(-1)
		MOV TMR2H,#HIGH(-1)
		SETB ET2				
		SETB TR2
		SETB EA
	M_LOOP:
		MOV P2,#083H
		CALL ROT_DELAY500
		MOV P2,#0FFH
		CALL ROT_DELAY500
		JB P1.7,NEXT_STATE_BLOCKED_RECOVER	
		JMP M_LOOP
		NEXT_STATE_BLOCKED_RECOVER:
		MOV NEXT_STATE,#S_RECOVER
RET		


;------ ROT_RECOVER-----------
ROT_RECOVER:      				 ; A Rotina recover inicializa a palavra passe a 0000.
MOV R0,#DIGIT_1
MOV R2,#4
RECOVER_DIGITS:
	MOV A,#0
	XRL A,#123					;xor com 123 para obter o digito da passe encriptada
	MOVX @R0,A					;mover para a memória externa o digito
	INC R0						;incrementar r0 para este apontar para o proximo digito
	DJNZ R2,RECOVER_DIGITS		;decrementar r4 de 4 até zero para transformar  e guardar os 4 digitos
NEXT_STATE_RECOVER_LOCKED:
MOV NEXT_STATE,#S_LOCKED
RET
;------------SubRotina-Selecionar-4Digitos-----------------;
SUB_ROT_4DIGITS:
	MOV COUNT,#0 					;contador inicializado a zero
	MOV R0,#DIGIT_1 				;r0 toma o valor da memoria do primeiro digito no array de digitos que serão selecionados pelo utilizador
CICLO_FOR_4X:
	MOV A,COUNT 				;move o valor da contagem para A
	CLR C						;limpar carry para fazer a subtração
	SUBB A,#4					;verifica se ja foram selecionados 4 digitos
	JZ CICLE_END				;se ja tiverem sido selecionados 4 digitos a subrotina acaba
	MOV INDEX,#0					;index inicializado a zero
	ROT_DISPLAY:
 		MOV A,INDEX 			;ao premir K_SET A toma o vaor de index
		CALL ROT_DISPLAY7S  	;o valor de index que foi passado para o Acumulador aparece no display
	BD1_LCK:
		JB K_LOAD,BD3_LCK 
	BD2_LCK:
		JNB K_LOAD,BD2_LCK
		JB K_LOAD,BA1_LCK
	BD3_LCK:
		JB K_SET,BD1_LCK
	BD4_LCK:
		JNB K_SET,BD4_LCK
		JB K_SET,BA2_LCK 
	BA1_LCK:				;Quando K_Load é premido segnifica que o utilizador selecionou esse numero
		MOV P2,#0FFH
		CALL ROT_DELAY500
		MOV @R0,INDEX			;O Index é colocado na posiçao do array para a qual R0 aponta 
		INC R0					;incrementar r0 para que ele aponte para a posição seguinte
		INC COUNT
		JMP CICLO_FOR_4X		;volta para o inicio do ciclo para escolher o proximo valor
	BA2_LCK:					;ao carregar no K_SET o utilizador pretende incrementar o valor no Display
		MOV A,INDEX             ;A toma o valor de Index
		CLR C					;limpar carry para fazer a subtração
		SUBB A,#15				;verifica se o valor no display ja chegou a F
		INC INDEX				;incrementa Index para apresentar o proximo valor no display
		JZ CICLO_FOR_4X			;volta para o dislpay
		JMP ROT_DISPLAY
	CICLE_END:
	RET
	
;---------10-SEGUNDOS-DELAY---------------------------
ROT_DELAY_10S:
	PUSH ACC
	PUSH PSW
	MOV CKCON,#2
	MOV TMOD,#10H	; temporizador de 16 bit
	CLR TF1
	SETB TR1
	MOV R3,#100
	R_CONT_10S:
		MOV TL1,#LOW(-50000)
		MOV TH1,#HIGH(-50000)
		JNB TF1,$
		CLR TF1
		DJNZ R3,R_CONT_10S
		POP ACC
		POP PSW
RET
	
;----------5-SEGUNDOS-DELAY--------------------------
ROT_DELAY_5S:
	PUSH ACC
	PUSH PSW
	MOV CKCON,#2
	MOV TMOD,#10H	; temporizador de 16 bit
	CLR TF1
	SETB TR1
	MOV R3,#100
	R_CONT_5S:
		MOV TL1,#LOW(-50000)
		MOV TH1,#HIGH(-50000)
		JNB TF1,$
		CLR TF1
		DJNZ R3,R_CONT_5S
		POP ACC
		POP PSW
RET
		
;---------------ROTINA-DE-DELAY-500MS-------------------------
ROT_DELAY500:
	PUSH ACC
	PUSH PSW
	MOV CKCON,#2
	MOV TMOD,#10H	; temporizador de 16 bit
	CLR TF1
	SETB TR1
	MOV R3,#10
RDEL_LOOP:
    MOV TL1,#LOW(-50000)	; reload tem de ser por instruções
	MOV TH1,#HIGH(-50000)
    JNB TF1,$
	CLR TF1
	DJNZ R3,RDEL_LOOP
	POP PSW
	POP ACC
RET
;---------- DISPLAY --------------------
ROT_DISPLAY7S:
	MOV DPTR,#1000H
	MOVC A, @A+DPTR
	MOV P2, A
	RET
CSEG AT 1000H 
DB 0C0H, 0F9H, 0A4H, 0B0H, 099H, 092H, 082H, 0F8H, 080H, 090H,088H,083H,0C6H,0A1H,086H,08EH
	END