

char Startup_R1[40] = "2,1,2_2,3_1,4_3,5_2,8_3,0_0,0_0\n";
char Startup_R2[40] = "2,2,1_2,3_2,4_2,5_1,6_4,0_0,0_0\n";
char Startup_R3[40] = "2,3,1_1,2_2,4_2,5_3,8_3,0_0,0_0\n";
char Startup_R4[40] = "2,4,1_3,2_2,3_2,5_3,6_4,8_4,0_0\n";
char Startup_R5[40] = "2,5,3_2,4_4,2_3,5_1,6_4,0_0,0_0\n";

char Startup_R6[40] = "2,6,2_4,4_4,5_4,7_3,0_0,0_0,0_0\n";
char Startup_R7[40] = "2,7,6_3,0_0,0_0,0_0,0_0,0_0,0_0\n";
char Startup_R8[40] = "2,8,1_3,3_3,4_4,0_0,0_0,0_0,0_0\n";


char Data_R1[35] = "4,1,11\n";
char Data_R2[35] = "4,2,22\n";
char Data_R3[30] = "4,3,33\n";
char Data_R4[35] = "4,4,44\n";
char Data_R5[35] = "4,5,55\n";

char Data_R6[30] = "4,6,66\n";
char Data_R7[20] = "7,ERROR\n";




char Data_R[20] = "4,_,MCA,6,27,100\n";
String Message;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  //Serial.println("Startup");

}

void loop() {
  // put your main code here, to run repeatedly:
  
  if (Serial.available()>0)
       {
        Message = Serial.readString();
        delay(100);
        if (Message[0] == '1')
            {
              if (Message[2] == '1')
                  {
                    Serial.write(Startup_R1);
                  }
              else if (Message[2] == '2')
                  {
                    Serial.write(Startup_R2);
                  }
              else if (Message[2] == '3')
                  {
                    Serial.write(Startup_R3);
                  }
              else if (Message[2] == '4')
                  {
                    Serial.write(Startup_R4);
                  }
              else if (Message[2] == '5')
                  {
                    Serial.write(Startup_R5);
                  }
              else if (Message[2] == '6')
                  {
                    Serial.write(Startup_R6);
                  }
              else if (Message[2] == '7')
                  {
                    Serial.write(Startup_R7);
                  }
              else if (Message[2] == '8')
                  {
                    Serial.write(Startup_R8);
                  }

              else
                  {
                    Serial.write("Error");
                    }
              
            }
        else if (Message[0] == '3' )
            {
              Data_R[2] = Message[2];
              if (Message[2] == '1')
                  {
                    Serial.write(Data_R1);
                  }
              else if (Message[2] == '2')
                  {
                    Serial.write(Data_R2);
                  }
              else if (Message[2] == '3')
                  {
                    Serial.write(Data_R3);
                  }
              else if (Message[2] == '4')
                  {
                    Serial.write(Data_R4);
                  }
              else if (Message[2] == '5')
                  {
                    Serial.write(Data_R5);
                  }
              else if (Message[2] == '6')
                  {
                    Serial.write(Data_R6);
                  }
              else if (Message[2] == '7')
                  {
                    Serial.write(Data_R7);
                  }

              else
                  {
                    Serial.write("Error");
                    }
             }
        else{
          Serial.write("Error");
          }
        }
}

