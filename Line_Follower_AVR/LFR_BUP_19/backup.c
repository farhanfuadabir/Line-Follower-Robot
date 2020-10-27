// /*
//  * sonar.c
//  *
//  * Created: 06-Apr-19 9:37:20 AM
//  *  Author: farha
//  */ 
// 
// 
// // 		Serial_sendInt(sonar1Read(), DEC);
// // 		Serial_sendString("\t");
// // 		_delay_ms(50);
// // 		Serial_sendInt(sonar2Read(), DEC);
// // 		Serial_sendString("\n");
// //
// // 		sonar2_val[sonar2_turn] = sonar2Read();
// // 		if (sonar2_turn >= 4)
// // 		{
// // 			sonar2_turn = 0;
// // 			uint8_t i = 0, j = 0, key = 0;
// //
// // 			for(i = 1; i < 5; i++)
// // 			{
// // 				key = sonar2_val[i];
// // 				j = i - 1;
// // 				while(j >= 0 && sonar2_val[j] > key)
// // 				{
// // 					sonar2_val[j + 1] = sonar2_val[j];
// // 					j--;
// // 				}
// // 				sonar2_val[j + 1] = key;
// // 			}
// //
// // 			distance2 = sonar2_val[4];
// // 		}
// // 		else sonar2_turn++;
// //
// //
// // 		if (distance2 < 12 && error < 2 && error > -2 )
// // 		{
// // 			Backward();
// // 			_delay_ms(100);
// // 			Stop();
// // 			while (distance2 < 12)
// // 			{
// // 				sonar2_val[sonar2_turn] = sonar2Read();
// // 				if (sonar2_turn >= 4)
// // 				{
// // 					sonar2_turn = 0;
// // 					uint8_t i = 0, j = 0, key = 0;
// //
// // 					for(i = 1; i < 5; i++)
// // 					{
// // 						key = sonar2_val[i];
// // 						j = i - 1;
// // 						while(j >= 0 && sonar2_val[j] > key)
// // 						{
// // 							sonar2_val[j + 1] = sonar2_val[j];
// // 							j--;
// // 						}
// // 						sonar2_val[j + 1] = key;
// // 					}
// //
// // 					distance2 = sonar2_val[2];
// // 				}
// // 				else sonar2_turn++;
// // 			}
// // 		}
// //
// //
// // 		if (sensorCount == 0 && sonar1Read() < 10)
// // 		{
// // 			distance1 = sonar1Read();
// // 			setRotationForward();
// // 			enable1(120);
// // 			enable2(120);
// // 			while (sensorCount == 0)
// // 			{
// // 				/**********************Hole Detection************************/
// //
// // 				if (PINB & (1 << PINB2))
// // 				{
// // 					Backward();
// // 					_delay_ms(50);
// // 					Stop();
// // 					setRotationForward();
// // 					while ((PINB & (1 << PINB2)));
// // 					_delay_ms(4000);
// // 					Forward();
// // 				}
// //
// // 				sonarError = sonar1Read() - distance1;
// // 				enable1(160 + (sonarError * 50));
// // 				enable2(160 - (sonarError * 50));
// // 				_delay_ms(20);
// // 				if (sonar2Read() < 7)
// // 				{
// // 					Backward();
// // 					_delay_ms(200);
// // 					Stop();
// // 					LeftSharp(130);
// // 					_delay_ms(500);
// // 					while (sonar1Read() > distance1);
// // 					setRotationForward();
// // 					sonarError = sonar1Read() - distance1;
// // 					enable1(120 + (sonarError * 40));
// // 					enable2(120 - (sonarError * 40));
// // 				}
// // 				sensorValueConversion();
// // 			}
// // 		}
// //
// // 		sonar2_val[sonar2_turn] = sonar2Read();
// // 		if (sonar2_turn >= 4)
// // 		{
// // 			sonar2_turn = 0;
// // 			uint8_t i = 0, j = 0, key = 0;
// //
// // 			for(i = 1; i < 5; i++)
// // 			{
// // 				key = sonar2_val[i];
// // 				j = i - 1;
// // 				while(j >= 0 && sonar2_val[j] > key)
// // 				{
// // 					sonar2_val[j + 1] = sonar2_val[j];
// // 					j--;
// // 				}
// // 				sonar2_val[j + 1] = key;
// // 			}
// //
// // 			distance2 = sonar2_val[2];
// // 		}
// // 		else sonar2_turn++;
// 
// // 		sonar1_val[sonar1_turn] = sonar1Read();
// // // 		Serial_sendInt(sonar1_val[sonar1_turn], DEC);
// // // 		Serial_sendString("\t\t\t");
// // 		if (sonar1_turn >= 4)
// // 		{
// // 			sonar1_turn = 0;
// // 			uint8_t i = 0, j = 0, key = 0;
// //
// // 			for(i = 1; i < 5; i++)
// // 			{
// // 				key = sonar1_val[i];
// // 				j = i - 1;
// // 				while(j >= 0 && sonar1_val[j] > key)
// // 				{
// // 					sonar1_val[j + 1] = sonar1_val[j];
// // 					j--;
// // 				}
// // 				sonar1_val[j + 1] = key;
// // 			}
// //
// // 			distance1 = sonar1_val[3];
// // // 			Serial_sendString("\n\n\n");
// // // 			Serial_sendInt(distance1, DEC);
// // // 			Serial_sendString("\t\t\t");
// // 		}
// // 		else sonar1_turn++;
// //
// // 		_delay_ms(50);
// //
// 
// //
// // 		if (distance2 != 111 && distance2 != 112 && distance2 < 10)
// // 		{
// // 			Backward();
// // 			_delay_ms(70);
// // 			setRotationForward();
// // 			Stop();
// // // 			Serial_sendInt(distance2, DEC);
// // // 			Serial_sendString("\n");
// //
// // // 			sonarError = distance2 - sonarSetPoint;
// // //
// // // 			while (sonarError != 0)
// // // 			{
// // // 				sonarError = distance2 - sonarSetPoint;
// // // 				prev_sonarError = sonarError;
// // // 				sonarCorrection = (sKp * sonarError) + (sKd * (sonarError - prev_sonarError));
// // //
// // // 				if (sonarCorrection >= 0) Forward();
// // // 				else if (sonarCorrection < 0)
// // // 				{
// // // 					Backward();
// // // 					sonarCorrection = - sonarCorrection;
// // // 				}
// // // 				if (sonarCorrection > maxSpeed)
// // // 				{
// // // 					enable1(maxSpeed);
// // // 					enable2(maxSpeed);
// // // 				}
// // // 				else
// // // 				{
// // // 					enable1(sonarCorrection);
// // // 					enable2(sonarCorrection);
// // // 				}
// // // 			}
// //
// // 			while (distance2 != 111 && distance2 != 112 && distance2 < 10)
// // 			{
// //
// // 			Serial_sendInt(distance2, DEC);
// // 			Serial_sendString("\n");
// //
// // 				sonar2_val[sonar2_turn] = sonar2Read();
// // 				if (sonar2_turn >= 4)
// // 				{
// // 					sonar2_turn = 0;
// // 					uint8_t i = 0, j = 0, key = 0;
// //
// // 					for(i = 1; i < 5; i++)
// // 					{
// // 						key = sonar2_val[i];
// // 						j = i - 1;
// // 						while(j >= 0 && sonar2_val[j] > key)
// // 						{
// // 							sonar2_val[j + 1] = sonar2_val[j];
// // 							j--;
// // 						}
// // 						sonar2_val[j + 1] = key;
// // 					}
// //
// // 					distance2 = sonar2_val[2];
// // 				}
// // 				else sonar2_turn++;
// // 			}
// // 		}
// 
// // 		while (sensorCount == 0)
// // 		{
// // 			distance1 = sonar1Read();
// // 			distance2 = sonar2Read();
// //
// // 			if (distance1 > 9 && distance1 < 60 && distance2 > 5)
// // 			{
// // 				enable1(0);
// // 				enable2(0);
// //
// // 				motor1a_PORT |= (1 << motor1a_PIN);
// // 				motor1b_PORT &= ~(1 << motor1b_PIN);
// // 				motor2a_PORT |= (1 << motor2a_PIN);
// // 				motor2b_PORT &= ~(1 << motor2b_PIN);
// //
// // 				enable1(50);
// // 				enable2(120);
// // 			}
// // 			else if (distance1 < 9 && distance2 > 5)
// // 			{
// // 				enable1(0);
// // 				enable2(0);
// //
// // 				motor1a_PORT |= (1 << motor1a_PIN);
// // 				motor1b_PORT &= ~(1 << motor1b_PIN);
// // 				motor2a_PORT |= (1 << motor2a_PIN);
// // 				motor2b_PORT &= ~(1 << motor2b_PIN);
// //
// // 				enable1(50);
// // 				enable2(120);
// // 			}
// //
// // 			else if (distance1 == 9 && distance2 > 5)
// // 			{
// // 				Forward();
// // 				enable1(140);
// // 				enable2(140);
// // 			}
// //
// // 			else if (distance2 <= 5)
// // 			{
// // 				Backward();
// // 				_delay_ms(80);
// // 				LeftSharp(120);
// // 				_delay_ms(500);
// // 				Forward();
// // 				Forward();
// // 				enable1(140);
// // 				enable2(140);
// //  				_delay_ms(100);
// // 			}
// //
// // 			else
// // 			{
// // 				Forward();
// // 				enable1(140);
// // 				enable2(140);
// // 			}
// //
// // 			sensorValueConversion();
// // 		}
// 
// 
// 
// // 		Serial_sendInt(distance1, DEC);
// // 		Serial_sendString("\t");
// //
// // 		_delay_ms(50);
// //
// // 		Serial_sendInt(distance2, DEC);
// // 		Serial_sendString("\n");
// 
// // 		sensorValueConversion();
// //
// // 		if (distance2 < 20)
// // 		{
// // 			Backward();
// // 			_delay_ms(50);
// // 			Stop();
// // 			while (distance2 < 11)
// // 			{
// // 				distance2 = sonar2Read();
// // 			}
// //
// // 		}
// 
// 
// 
// 
// 
// 
// 
// 
// 
// 
// 
// 
// 
// 
// 
// 
// 
// 
// 
// 
// 		if (leftFlag == 1 && weightedValue < 7 && weightedValue > 3)
// 		{
// 			Forward();
// 			while (leftFlag == 1)
// 			{
// 				sensorValueConversion();
// 				if (allFlag == 1 || rightFlag == 1) break;
// 			}
// 
// 			if (allFlag == 0 && rightFlag == 0)
// 			{
// 				_delay_ms(150);
// 				sensorValueConversion();
// 				if (sensorCount == 0 && rightFlag == 0)
// 				{
// 					Backward();
// 					_delay_ms(250);
// 					RightSharp(150);
// 					_delay_ms(300);
// 					sensorValueConversion();
// 				}
// 				if (sensorCount == 0 && rightFlag == 0)
// 				{
// 					LeftSharp(140);
// 					_delay_ms(350);
// 					while (sensorCount == 0 || weightedValue < 3)
// 					{
// 						sensorValueConversion();
// 						if (rightFlag == 1) break;
// 					}
// 					// 					Backward();
// 					// 					_delay_ms(400);
// 					// 					setRotationForward();
// 					sensorValueConversion();
// 				}
// 			}
// 		}
// 
// 		else if (rightFlag == 1 && weightedValue < 7 && weightedValue > 3)
// 		{
// 			Forward();
// 			while (rightFlag == 1)
// 			{
// 				sensorValueConversion();
// 				if (allFlag == 1 || leftFlag == 1) break;
// 			}
// 			
// 			if (allFlag == 0 && leftFlag == 0)
// 			{
// 				_delay_ms(80);
// 				// 				sensorValueConversion();
// 
// 				RightSharp(140);
// 				//				_delay_ms(100);
// 				
// 				while (rightFlag == 0 || sensorCount == 0 || weightedValue > 7 || weightedValue < 4) sensorValueConversion();
// 			}
// 		}
// 		
// 		else if (allFlag == 1 && weightedValue < 7 && weightedValue > 3)
// 		{
// 			Forward();
// 			while (allFlag == 1 && stopFlag < 100) sensorValueConversion();
// 
// 			if (stopFlag >= 100) // work with stopflag
// 			{
// 				Backward();
// 				_delay_ms(100);
// 				Stop();
// 				_delay_ms(2000);
// 				while (sensorCount == 6 && allFlag == 1) sensorValueConversion();
// 			}
// 
// 			// 			else
// 			// 			{
// 			// 				_delay_ms(80);
// 			// 				sensorValueConversion();
// 			//
// 			// 				RightSharp(140);
// 			// 				_delay_ms(100);
// 			//
// 			// 				while (rightFlag == 0 || sensorCount == 0 || weightedValue > 7 || weightedValue < 4)	sensorValueConversion();
// 			// 			}
// 		}















if(leftFlag == 1 && weightedValue < 7 && weightedValue > 3)
{
	Forward();
	while (leftFlag == 1)
	{
		sensorValueConversion();
		if (allFlag == 1 || rightFlag == 1) break;
	}
	if (allFlag == 0 && rightFlag == 0)
	{
		_delay_ms(150);
		LeftSharp(140);
		if (sensorCount == 0 && rightFlag == 0)
		{
			LeftSharp(140);
			_delay_ms(100);
			while (sensorCount == 0 || weightedValue < 3) sensorValueConversion();
		}
	}

	if(rightFlag == 1 && weightedValue < 7 && weightedValue > 3)
	{
		Forward();
		while (rightFlag == 1)
		{
			sensorValueConversion();
			if (allFlag == 1 || leftFlag == 1) break;
		}
		if (allFlag == 0 && leftFlag == 0)
		{
			_delay_ms(150);
			RightSharp(140);
			if (sensorCount == 0 && rightFlag == 0)
			{
				RightSharp(140);
				_delay_ms(100);
				while (sensorCount == 0 || weightedValue > 7) sensorValueConversion();
			}
		}
	}

	if (allFlag == 1 && weightedValue < 7 && weightedValue > 3)
	{
		Forward();
		while (allFlag == 1 && stopFlag < 100) sensorValueConversion();

		if (stopFlag >= 100) // work with stopflag
		{
			Backward();
			_delay_ms(100);
			Stop();
			_delay_ms(2000);
			while (sensorCount == 6 && allFlag == 1) sensorValueConversion();
		}
	}