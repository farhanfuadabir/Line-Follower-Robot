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
// 				Stop();
// 				_delay_ms(1000);
				_delay_ms(150);
				sensorValueConversion();
				LeftSharp(160);
				if (sensorCount == 0 && rightFlag == 0)
				{
					Backward();
					_delay_ms(350);
					LeftSharp(120);
					_delay_ms(100);
					while (sensorCount == 0 || weightedValue < 5) sensorValueConversion();
				}
			}
		}

		else if(rightFlag == 1 && weightedValue < 7 && weightedValue > 3)
		{
			Forward();
			while (rightFlag == 1)
			{
				sensorValueConversion();
				if (allFlag == 1 || leftFlag == 1) break;
			}
			if (allFlag == 0 && leftFlag == 0)
			{
// 				Stop();
// 				_delay_ms(1000);
				_delay_ms(150);
				sensorValueConversion();
				RightSharp(160);
				if (sensorCount == 0 && leftFlag == 0)
				{
					Backward();
					_delay_ms(350);
					RightSharp(120);
					_delay_ms(200);
					while (sensorCount == 0 || weightedValue > 5) sensorValueConversion();
				}
			}
		}

		else if (allFlag == 1 && weightedValue < 7 && weightedValue > 3)
		{
// 			Stop();
// 			_delay_ms(2000);
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