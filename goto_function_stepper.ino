const int stepPin_2 = 10; 
const int dirPin_2= 11; 


const int stepPin_1 = 6;                          //8
const int dirPin_1= 7;                              //9

const int led_Pin=13;
int stepper(int, int, int, int);           // steps  speed  direction   motor

double current_1=0;
double current_2=0;

double theta1=0, theta2=0;

double line_x[200];
double line_y[200];

void setup() {
  // Sets the two pins as Outputs
  pinMode(stepPin_1,OUTPUT); 
  pinMode(dirPin_1,OUTPUT);
  pinMode(stepPin_2,OUTPUT); 
  pinMode(dirPin_2,OUTPUT);
  pinMode(led_Pin,OUTPUT);
int m=0;
Serial.begin(9600);
//stepper(1,1000,0,0);
//stepper(1,1000,0,1);
digitalWrite(led_Pin,LOW);


}



void loop() {
int temp_steps=679/9;
digitalWrite(led_Pin,HIGH);
if(Serial.available()>0)
{char s='p';
s=Serial.read();
if(s=='q')
{for( int i=0; i<temp_steps; i++)
{stepper(1,1500,0,0);
digitalWrite(led_Pin,HIGH);}
s='p';
}

if(s=='r')
{for( int i=0; i<temp_steps; i++)
{stepper(1,1500,1,0);
digitalWrite(led_Pin,HIGH);}
s='p';
}

if(s=='s')
{for( int i=0; i<temp_steps; i++)
 {stepper(1,1500,1,1);
digitalWrite(led_Pin,HIGH);}
s='p';}

if(s=='d')
{for( int i=0; i<temp_steps; i++)
  {stepper(1,1500,0,1);
digitalWrite(led_Pin,HIGH);}
s='p';}

if(s=='z')
{goto_func(370,0);
s='p';}

if(s=='x')
{goto_func(0,-370);
s='p';}

if(s=='f')
{goto_func(260,260);
s='p';}

if(s=='l')
{draw_line(370,0,200,0);
s='p';
} 
/*stepper(97,3000,1);
delay(3000);
stepper(97,3000,0);
delay(3000);*/

//stepper(97,800,0,0);
//stepper(97,800,0,1);

//Serial.print(current_1/15.08888);Serial.print("          ");Serial.println(current_2/15.088888);

}
}


int stepper( int steps, int rpm, int rotn_direc, int motor)
{int m=0;
digitalWrite(dirPin_1,rotn_direc);
digitalWrite(dirPin_2,rotn_direc);
int temp=89;

if(motor==0)
{temp=stepPin_1;
if(rotn_direc==0)
{current_1=current_1-steps;}
if(rotn_direc==1)
{current_1=current_1+steps;}
}

if(motor==1)
{temp=stepPin_2;
if(rotn_direc==0)
{current_2=current_2+steps;}
if(rotn_direc==1)
{current_2=current_2-steps;}
}
//Serial.print("c");Serial.print("  ");Serial.print(current_1);Serial.print("  ");Serial.println(current_2);
//Serial.println(temp);

while(m<steps)
{   digitalWrite(temp,HIGH);
    delayMicroseconds(rpm);
    digitalWrite(temp,LOW);
    delayMicroseconds(rpm);
  m++;
  }
}


int inv_kinematics(double x1, double y1)
{  
 theta2=(acos(((x1*x1)+(y1*y1)-72148)/32376/2));
 theta1=(atan2(y1,x1)-atan2(228*sin(theta2),(142+(228*cos(theta2)))));         //getting values of joint angles
theta2=theta2*180/3.142;    theta1=theta1*180/3.142;
//Serial.print(theta1);Serial.print("      ");Serial.print(theta2);Serial.print("    ");Serial.print(x1);Serial.print("    ");Serial.println(y1);
}



int goto_func(int x , int y)

{//double temp=0;
//temp=(x*x)+(y*y);
//if(temp>136900)
//{x=370;y=0;}
 inv_kinematics(x,y);
int target_1=theta1*679/45;
int target_2=theta2*679/45;


int temp_rotn_direc_1=0;
int temp_rotn_direc_2=1;
if( target_1-current_1>=0  )
{temp_rotn_direc_1=1;}                              //change to proper val
if( target_2-current_2>0)
{temp_rotn_direc_2=0;}
int flag_1=0, flag_2=0;
while(1)
{
  if((current_1)!=(target_1)   &&  abs(current_1-target_1)>0.1)
  {stepper(1,1000,temp_rotn_direc_1,0);}
  else
  {flag_1=1;}
 if(abs(current_2)!=abs(target_2)    &&   abs(current_2-target_2)>0.1)
  {stepper(1,1000,temp_rotn_direc_2,1);}
  else
  {flag_2=1;}
  if(flag_1==1    &&    flag_2==1)
  {break;}
}}


int draw_line( double x1, double y1, double x2, double y2)
{ double slope=0;
  
   {slope=(y2-y1)/(x2-x1);}
  Serial.println(slope);
  if(slope>=1)
  {for(int i=0; i<abs(y2-y1); i=i+1)
  {if(y2>y1)
    {line_y[i]=y1+i;}
   else
    {line_y[i]=y1-i;}
  line_x[i]=((1/slope)*(line_y[i]-y1))+x1;
  //Serial.print(line_x[i]);Serial.print("        ");Serial.println(line_y[i]);
  goto_func(line_x[i],line_y[i]);}
  }
  
  else
  {for(int i=0; i<abs(x2-x1); i=i+1)
  {if(x2>x1)
    {line_x[i]=x1+i;}
   else
    {line_x[i]=x1-i;}
  line_y[i]=((slope)*(line_x[i]-x1))+y1;
  //Serial.print(line_x[i]);Serial.print("        ");Serial.println(line_y[i]);
  goto_func(line_x[i],line_y[i]);}
  } 
} 
