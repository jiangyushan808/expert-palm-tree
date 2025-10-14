#include <main.h>
#include <tim.h>
#include <encoder.h>
#include <motor.h>
long Integral_bias_Left = 0 ;
<<<<<<< HEAD
long Integral_bias_Right = 0 ; /* åå·®ç´¯ç§¯ */
=======
long Integral_bias_Right = 0 ; /* Æ«²îÀÛ»ı */
>>>>>>> 8a274eeea5e1c58bf1cf451062b760e3f76587d0

float Bias_Left = 0,Last_bias_Left = 0,Prev_bias_Left = 0;
float Bias_Right = 0,Last_bias_Right = 0,Prev_bias_Right = 0;
float Bias_yaw = 0,Last_bias_yaw = 0;
long Integral_bias_yaw = 0;

<<<<<<< HEAD
/* ä½ç½®ç¯PIDç³»æ•° */
=======
/* Î»ÖÃ»·PIDÏµÊı */
>>>>>>> 8a274eeea5e1c58bf1cf451062b760e3f76587d0

float Position_KP_L=0.13,Position_KI_L=0.00112,Position_KD_L=1.001;//6.382 
float Position_KP_R=0.13,Position_KI_R=0.00112,Position_KD_R=1.001;//6.382 


<<<<<<< HEAD
/*å†…ç¯ é€Ÿåº¦ç¯*/
float Incremental_KP_L =1.65,Incremental_KI_L =  0.275, Incremental_KD_L = 0.1;  //1.65ï¼Œ0.275ï¼Œ0.1
float Incremental_KP_R = 1.65,Incremental_KI_R = 0.275,Incremental_KD_R = 0.1; 

/*********************è§’åº¦ç¯1*********************/
float Yaw_KP = 8, Yaw_KI = 0 ,Yaw_KD = 0;
int Yaw_PWM;
float Yaw_fAngle = 0.0f;
/*********************è§’åº¦ç¯*********************/
=======
/*ÄÚ»· ËÙ¶È»·*/
float Incremental_KP_L =1.65,Incremental_KI_L =  0.275, Incremental_KD_L = 0.1;  //1.65£¬0.275£¬0.1
float Incremental_KP_R = 1.65,Incremental_KI_R = 0.275,Incremental_KD_R = 0.1; 

/*********************½Ç¶È»·1*********************/
float Yaw_KP = 10, Yaw_KI = 0 ,Yaw_KD = 0;
int Yaw_PWM;
float Yaw_fAngle = 0.0f;
/*********************½Ç¶È»·*********************/
>>>>>>> 8a274eeea5e1c58bf1cf451062b760e3f76587d0




/**************************************************************************
<<<<<<< HEAD
 * @brief  ä½ç½®å¼PIDæ§åˆ¶å™¨
 * @param  å®é™…ä½ç½®realityï¼Œç›®æ ‡ä½ç½®target
 * @retval ç”µæœºPWM
 * @attention
 * @describe æ ¹æ®ä½ç½®å¼ç¦»æ•£PIDå…¬å¼
				pwm=Kp*e(k)+Ki*âˆ‘e(k)+Kd[eï¼ˆkï¼‰-e(k-1)]
				e(k)ä»£è¡¨æœ¬æ¬¡åå·®
				e(k-1)ä»£è¡¨ä¸Šä¸€æ¬¡çš„åå·®
				âˆ‘e(k)ä»£è¡¨e(k)ä»¥åŠä¹‹å‰çš„åå·®çš„ç´¯ç§¯å’Œ;å…¶ä¸­kä¸º1,2,...,k;
 * @matters pwmä»£è¡¨è¾“å‡º
**************************************************************************/
long Position_PID_Left(long reality, long target,int reset) //new: reset
{
	static long Bias, Last_Bias;   // åå·®Biasï¼Œä¸Šæ¬¡åå·®Last_Bias
	static long PWM_OUT;

	
	// new: æ·»åŠ é‡ç½®åŠŸèƒ½   
	//é‡ç½®æ¨¡å¼ - å½“reset=1æ—¶æ‰§è¡Œ
=======
 * @brief  Î»ÖÃÊ½PID¿ØÖÆÆ÷
 * @param  Êµ¼ÊÎ»ÖÃreality£¬Ä¿±êÎ»ÖÃtarget
 * @retval µç»úPWM
 * @attention
 * @describe ¸ù¾İÎ»ÖÃÊ½ÀëÉ¢PID¹«Ê½
				pwm=Kp*e(k)+Ki*¡Æe(k)+Kd[e£¨k£©-e(k-1)]
				e(k)´ú±í±¾´ÎÆ«²î
				e(k-1)´ú±íÉÏÒ»´ÎµÄÆ«²î
				¡Æe(k)´ú±íe(k)ÒÔ¼°Ö®Ç°µÄÆ«²îµÄÀÛ»ıºÍ;ÆäÖĞkÎª1,2,...,k;
 * @matters pwm´ú±íÊä³ö
**************************************************************************/
long Position_PID_Left(long reality, long target,int reset) //new: reset
{
	static long Bias, Last_Bias;   // Æ«²îBias£¬ÉÏ´ÎÆ«²îLast_Bias
	static long PWM_OUT;

	
	// new: Ìí¼ÓÖØÖÃ¹¦ÄÜ   
	//ÖØÖÃÄ£Ê½ - µ±reset=1Ê±Ö´ĞĞ
>>>>>>> 8a274eeea5e1c58bf1cf451062b760e3f76587d0
    if(reset == 1) {
        Bias = 0;
        Last_Bias = 0;
        PWM_OUT = 0;
			Integral_bias_Left = 0;
<<<<<<< HEAD
        return 0; // è¿”å›å€¼ä¸é‡è¦ï¼Œ reset æ¨¡å¼ä»…ç”¨äºæ¸…é›¶ï¼Œä¸å‚ä¸å®é™…è¾“å‡º
    }
	
	
	Bias = target - reality; /* è®¡ç®—åå·® */
	Integral_bias_Left += Bias;	 /* åå·®ç´¯ç§¯ */

	if (Integral_bias_Left > 5000)
		Integral_bias_Left = 5000; /* ç§¯åˆ†é™å¹… */
	if (Integral_bias_Left < -5000)
		Integral_bias_Left = -5000;

	PWM_OUT = (Position_KP_L * Bias)					/* æ¯”ä¾‹ç¯èŠ‚ */
			  + (Position_KI_L * Integral_bias_Left)		/* ç§¯åˆ†ç¯èŠ‚ */
			  + (Position_KD_L * (Bias - Last_Bias)); /* å¾®åˆ†ç¯èŠ‚ */

	Last_Bias = Bias; /* ä¿å­˜ä¸Šæ¬¡åå·® */

	return PWM_OUT; /* è¾“å‡ºç»“æœ */
}

//å³è½®çš„ä½ç½®å¼PID --- å†™ä¸¤ä¸ªå‡½æ•°ï¼Œæ˜¯ä¸ºäº†ç¼–ç å™¨æ•°æ®ä¸å¹²æ‰°
=======
        return 0; // ·µ»ØÖµ²»ÖØÒª£¬ reset Ä£Ê½½öÓÃÓÚÇåÁã£¬²»²ÎÓëÊµ¼ÊÊä³ö
    }
	
	
	Bias = target - reality; /* ¼ÆËãÆ«²î */
	Integral_bias_Left += Bias;	 /* Æ«²îÀÛ»ı */

	if (Integral_bias_Left > 5000)
		Integral_bias_Left = 5000; /* »ı·ÖÏŞ·ù */
	if (Integral_bias_Left < -5000)
		Integral_bias_Left = -5000;

	PWM_OUT = (Position_KP_L * Bias)					/* ±ÈÀı»·½Ú */
			  + (Position_KI_L * Integral_bias_Left)		/* »ı·Ö»·½Ú */
			  + (Position_KD_L * (Bias - Last_Bias)); /* Î¢·Ö»·½Ú */

	Last_Bias = Bias; /* ±£´æÉÏ´ÎÆ«²î */

	return PWM_OUT; /* Êä³ö½á¹û */
}

//ÓÒÂÖµÄÎ»ÖÃÊ½PID --- Ğ´Á½¸öº¯Êı£¬ÊÇÎªÁË±àÂëÆ÷Êı¾İ²»¸ÉÈÅ
>>>>>>> 8a274eeea5e1c58bf1cf451062b760e3f76587d0

long Position_PID_Right(long reality, long target,int reset)

{
<<<<<<< HEAD
	static long Bias, Last_Bias;   // åå·®Biasï¼Œä¸Šæ¬¡åå·®Last_Bias
	static long PWM_OUT;


	// new: æ·»åŠ é‡ç½®åŠŸèƒ½   
	//é‡ç½®æ¨¡å¼ - å½“reset=1æ—¶æ‰§è¡Œ
=======
	static long Bias, Last_Bias;   // Æ«²îBias£¬ÉÏ´ÎÆ«²îLast_Bias
	static long PWM_OUT;


	// new: Ìí¼ÓÖØÖÃ¹¦ÄÜ   
	//ÖØÖÃÄ£Ê½ - µ±reset=1Ê±Ö´ĞĞ
>>>>>>> 8a274eeea5e1c58bf1cf451062b760e3f76587d0
    if(reset == 1) {
        Bias = 0;
        Last_Bias = 0;
        PWM_OUT = 0;
			Integral_bias_Right = 0;
<<<<<<< HEAD
        return 0; // è¿”å›å€¼ä¸é‡è¦ï¼Œ reset æ¨¡å¼ä»…ç”¨äºæ¸…é›¶ï¼Œä¸å‚ä¸å®é™…è¾“å‡º
=======
        return 0; // ·µ»ØÖµ²»ÖØÒª£¬ reset Ä£Ê½½öÓÃÓÚÇåÁã£¬²»²ÎÓëÊµ¼ÊÊä³ö
>>>>>>> 8a274eeea5e1c58bf1cf451062b760e3f76587d0
    }
	
	

<<<<<<< HEAD
	Bias = target - reality; /* è®¡ç®—åå·® */
	Integral_bias_Right += Bias;	 /* åå·®ç´¯ç§¯ */

	if (Integral_bias_Right > 5000)
		Integral_bias_Right = 5000; /* ç§¯åˆ†é™å¹… */
	if (Integral_bias_Right < -5000)
		Integral_bias_Right = -5000;

	PWM_OUT = (Position_KP_R * Bias)					/* æ¯”ä¾‹ç¯èŠ‚ */
			  + (Position_KI_R * Integral_bias_Right)		/* ç§¯åˆ†ç¯èŠ‚ */
			  + (Position_KD_R * (Bias - Last_Bias)); /* å¾®åˆ†ç¯èŠ‚ */

	Last_Bias = Bias; /* ä¿å­˜ä¸Šæ¬¡åå·® */
	return PWM_OUT; /* è¾“å‡ºç»“æœ */
=======
	Bias = target - reality; /* ¼ÆËãÆ«²î */
	Integral_bias_Right += Bias;	 /* Æ«²îÀÛ»ı */

	if (Integral_bias_Right > 5000)
		Integral_bias_Right = 5000; /* »ı·ÖÏŞ·ù */
	if (Integral_bias_Right < -5000)
		Integral_bias_Right = -5000;

	PWM_OUT = (Position_KP_R * Bias)					/* ±ÈÀı»·½Ú */
			  + (Position_KI_R * Integral_bias_Right)		/* »ı·Ö»·½Ú */
			  + (Position_KD_R * (Bias - Last_Bias)); /* Î¢·Ö»·½Ú */

	Last_Bias = Bias; /* ±£´æÉÏ´ÎÆ«²î */
	return PWM_OUT; /* Êä³ö½á¹û */
>>>>>>> 8a274eeea5e1c58bf1cf451062b760e3f76587d0
}




/**************************************************************************
<<<<<<< HEAD
 * @brief  å¢é‡å¼PIDæ§åˆ¶å™¨
 * @param  å®é™…å€¼realityï¼Œç›®æ ‡å€¼target
 * @retval ç”µæœºPWM
 * @attention
 * @describe æ ¹æ®å¢é‡å¼ç¦»æ•£PIDå…¬å¼ 
				pwm+=Kp[eï¼ˆkï¼‰-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
				e(k)ä»£è¡¨æœ¬æ¬¡åå·® 
				e(k-1)ä»£è¡¨ä¸Šä¸€æ¬¡çš„åå·®  ä»¥æ­¤ç±»æ¨ 
 * @matters pwmä»£è¡¨è¾“å‡º
=======
 * @brief  ÔöÁ¿Ê½PID¿ØÖÆÆ÷
 * @param  Êµ¼ÊÖµreality£¬Ä¿±êÖµtarget
 * @retval µç»úPWM
 * @attention
 * @describe ¸ù¾İÔöÁ¿Ê½ÀëÉ¢PID¹«Ê½ 
				pwm+=Kp[e£¨k£©-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
				e(k)´ú±í±¾´ÎÆ«²î 
				e(k-1)´ú±íÉÏÒ»´ÎµÄÆ«²î  ÒÔ´ËÀàÍÆ 
 * @matters pwm´ú±íÊä³ö
>>>>>>> 8a274eeea5e1c58bf1cf451062b760e3f76587d0
**************************************************************************/
int Incremental_PID_Left(int reality,int target, int reset)
{ 	
	 static float Pwm;
	
<<<<<<< HEAD
	// new: æ·»åŠ é‡ç½®åŠŸèƒ½  
	// é‡ç½®æ¨¡å¼
=======
	// new: Ìí¼ÓÖØÖÃ¹¦ÄÜ  
	// ÖØÖÃÄ£Ê½
>>>>>>> 8a274eeea5e1c58bf1cf451062b760e3f76587d0
    if (reset == 1) {
        Pwm = 0;
        Bias_Left = 0;
        Last_bias_Left = 0;
        Prev_bias_Left = 0;
        return 0;
    }
   
<<<<<<< HEAD
	 Bias_Left=target-reality;                                  /* è®¡ç®—åå·® */

		
	 Pwm += (Incremental_KP_L*(Bias_Left-Last_bias_Left))               /* æ¯”ä¾‹ç¯èŠ‚ */
           +Incremental_KI_L*Bias_Left                                              /* ç§¯åˆ†ç¯èŠ‚ */
           +(Incremental_KD_L*(Bias_Left - 2 * Last_bias_Left + Prev_bias_Left));  /* å¾®åˆ†ç¯èŠ‚ */ 
		
  
   Prev_bias_Left=Last_bias_Left;                                   /* ä¿å­˜ä¸Šä¸Šæ¬¡åå·® */
	 Last_bias_Left=Bias_Left;	                                    /* ä¿å­˜ä¸Šä¸€æ¬¡åå·® */
    
	 return Pwm;                                            /* è¾“å‡ºç»“æœ */
}
//å³è½® å¢é‡å¼PIDè®¡ç®—
=======
	 Bias_Left=target-reality;                                  /* ¼ÆËãÆ«²î */

		
	 Pwm += (Incremental_KP_L*(Bias_Left-Last_bias_Left))               /* ±ÈÀı»·½Ú */
           +Incremental_KI_L*Bias_Left                                              /* »ı·Ö»·½Ú */
           +(Incremental_KD_L*(Bias_Left - 2 * Last_bias_Left + Prev_bias_Left));  /* Î¢·Ö»·½Ú */ 
		
  
   Prev_bias_Left=Last_bias_Left;                                   /* ±£´æÉÏÉÏ´ÎÆ«²î */
	 Last_bias_Left=Bias_Left;	                                    /* ±£´æÉÏÒ»´ÎÆ«²î */
    
	 return Pwm;                                            /* Êä³ö½á¹û */
}
//ÓÒÂÖ ÔöÁ¿Ê½PID¼ÆËã
>>>>>>> 8a274eeea5e1c58bf1cf451062b760e3f76587d0

int Incremental_PID_Right(int reality,int target, int reset)
{ 	
	 static float Pwm;
	
<<<<<<< HEAD
	// new: æ·»åŠ é‡ç½®åŠŸèƒ½  
	// é‡ç½®æ¨¡å¼
=======
	// new: Ìí¼ÓÖØÖÃ¹¦ÄÜ  
	// ÖØÖÃÄ£Ê½
>>>>>>> 8a274eeea5e1c58bf1cf451062b760e3f76587d0
    if (reset == 1) {
        Pwm = 0;
        Bias_Right = 0;
        Last_bias_Right = 0;
        Prev_bias_Right = 0;
        return 0;
    }
	
	

<<<<<<< HEAD
	 Bias_Right=target-reality;                                   /* è®¡ç®—åå·® */
		
		
	 Pwm += (Incremental_KP_R*(Bias_Right-Last_bias_Right))               /* æ¯”ä¾‹ç¯èŠ‚ */
           +Incremental_KI_R*Bias_Right                      /* ç§¯åˆ†ç¯èŠ‚ */
           +(Incremental_KD_R*(Bias_Right - 2 * Last_bias_Right + Prev_bias_Right));  /* å¾®åˆ†ç¯èŠ‚ */ 

				
    
   Prev_bias_Right=Last_bias_Right;                                   /* ä¿å­˜ä¸Šä¸Šæ¬¡åå·® */
	 Last_bias_Right=Bias_Right;	                                    /* ä¿å­˜ä¸Šä¸€æ¬¡åå·® */
    
	 return Pwm;                                            /* è¾“å‡ºç»“æœ */
=======
	 Bias_Right=target-reality;                                   /* ¼ÆËãÆ«²î */
		
		
	 Pwm += (Incremental_KP_R*(Bias_Right-Last_bias_Right))               /* ±ÈÀı»·½Ú */
           +Incremental_KI_R*Bias_Right                      /* »ı·Ö»·½Ú */
           +(Incremental_KD_R*(Bias_Right - 2 * Last_bias_Right + Prev_bias_Right));  /* Î¢·Ö»·½Ú */ 

				
    
   Prev_bias_Right=Last_bias_Right;                                   /* ±£´æÉÏÉÏ´ÎÆ«²î */
	 Last_bias_Right=Bias_Right;	                                    /* ±£´æÉÏÒ»´ÎÆ«²î */
    
	 return Pwm;                                            /* Êä³ö½á¹û */
>>>>>>> 8a274eeea5e1c58bf1cf451062b760e3f76587d0
}




<<<<<<< HEAD
//è§’åº¦ç¯
=======
//½Ç¶È»·
>>>>>>> 8a274eeea5e1c58bf1cf451062b760e3f76587d0
int JY61P_Yaw_correct(float current_yaw, float target_yaw,int reset)

{
	
	static float PWM_OUT;
	
	
<<<<<<< HEAD
		// new: æ·»åŠ é‡ç½®åŠŸèƒ½   
	//é‡ç½®æ¨¡å¼ - å½“reset=1æ—¶æ‰§è¡Œ
=======
		// new: Ìí¼ÓÖØÖÃ¹¦ÄÜ   
	//ÖØÖÃÄ£Ê½ - µ±reset=1Ê±Ö´ĞĞ
>>>>>>> 8a274eeea5e1c58bf1cf451062b760e3f76587d0
    if(reset == 1) {
        Bias_yaw = 0;
        Last_bias_yaw = 0;
        PWM_OUT = 0;
			Integral_bias_yaw = 0;
<<<<<<< HEAD
        return 0; // è¿”å›å€¼ä¸é‡è¦ï¼Œ reset æ¨¡å¼ä»…ç”¨äºæ¸…é›¶ï¼Œä¸å‚ä¸å®é™…è¾“å‡º
    }
	
    Bias_yaw = target_yaw - current_yaw; /* è®¡ç®—åå·® */
		
//		//æŠŠè§’åº¦å·®å€¼é™å®šåœ¨-180~180ï¼Œå¸¦ç¯ç»•å¤„ç†
//    if (Bias_yaw > 180.0f) Bias_yaw -= 360.0f;
//    if (Bias_yaw < -180.0f) Bias_yaw += 360.0f;
		
	Integral_bias_yaw += Bias_yaw;	 /* åå·®ç´¯ç§¯ */
	//if(real_num == 2)
	//	Integral_bias_yaw=0;
	if (Integral_bias_yaw > 1500)
		Integral_bias_yaw = 1500; /* ç§¯åˆ†é™å¹… */
	if (Integral_bias_yaw < -1500)
		Integral_bias_yaw = -1500;

	PWM_OUT = (Yaw_KP * Bias_yaw)					/* æ¯”ä¾‹ç¯èŠ‚ */
			  + (Yaw_KI * Integral_bias_yaw)		/* ç§¯åˆ†ç¯èŠ‚ */
			  + (Yaw_KD * (Bias_yaw - Last_bias_yaw)); /* å¾®åˆ†ç¯èŠ‚ */

	Last_bias_yaw = Bias_yaw; /* ä¿å­˜ä¸Šæ¬¡åå·® */
	
	return PWM_OUT;
}
	
=======
        return 0; // ·µ»ØÖµ²»ÖØÒª£¬ reset Ä£Ê½½öÓÃÓÚÇåÁã£¬²»²ÎÓëÊµ¼ÊÊä³ö
    }
	
    Bias_yaw = target_yaw - current_yaw; /* ¼ÆËãÆ«²î */
		
		//°Ñ½Ç¶È²îÖµÏŞ¶¨ÔÚ-180~180£¬´ø»·ÈÆ´¦Àí
    if (Bias_yaw > 180.0f) Bias_yaw -= 360.0f;
    if (Bias_yaw < -180.0f) Bias_yaw += 360.0f;
		
	Integral_bias_yaw += Bias_yaw;	 /* Æ«²îÀÛ»ı */
	//if(real_num == 2)
	//	Integral_bias_yaw=0;
	if (Integral_bias_yaw > 1500)
		Integral_bias_yaw = 1500; /* »ı·ÖÏŞ·ù */
	if (Integral_bias_yaw < -1500)
		Integral_bias_yaw = -1500;

	PWM_OUT = (Yaw_KP * Bias_yaw)					/* ±ÈÀı»·½Ú */
			  + (Yaw_KI * Integral_bias_yaw)		/* »ı·Ö»·½Ú */
			  + (Yaw_KD * (Bias_yaw - Last_bias_yaw)); /* Î¢·Ö»·½Ú */

	Last_bias_yaw = Bias_yaw; /* ±£´æÉÏ´ÎÆ«²î */
	
	return PWM_OUT;
}
	
>>>>>>> 8a274eeea5e1c58bf1cf451062b760e3f76587d0
