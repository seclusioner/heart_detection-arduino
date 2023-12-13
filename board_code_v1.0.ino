/*
For computer version 

1. Arduino leonardo board
2. Serial Plot
3. Labview

*/

#include <SoftwareSerial.h>
SoftwareSerial MyBlue(9, 10); // RX | TX 

#define DATA_num 32
#define SAMPLE_FREQ 4
#define INPUT_PIN 18
#define DEL 3

const int ref=400;
unsigned int flag=0,bb=0,bbo=0;
int owo=0,owo1=0;
int data[DATA_num]={0};
int r=0,r1=0;
int v0=0,v1=0,v2=0,v3=0;
int sum=0;
float RR=0;
int samp0=0,samp1=0;
int count=0,flag1=0;
int owo3=0,owo4=0;
int inp;
int flag2=0;

//auto gain control
int adc_value=0;
int increase=0,decrease=0;
int samp2=0,samp3=0;
int adc_arry[500];
int adc_times=0;
int up=0,down=0;
int gain=2;//===================

//RR input
float RR_interval[DATA_num];
float delta_RR[DATA_num-2*DEL];
float new_RR[DATA_num-2*DEL];
int index=0;
int HR_RR[5];
int HR_times=0;
int HR_sum=0;

//alarm
int times=0;
int alarm=0;

//BT
int bt_counter=0;
int bt_counter1=0;

//FFT
int fft_data[16]={0};
float f_peaks[5]; // top 5 frequencies peaks in descending order
byte sine_data [91]=
{
0,  
4,    9,    13,   18,   22,   27,   31,   35,   40,   44, 
49,   53,   57,   62,   66,   70,   75,   79,   83,   87, 
91,   96,   100,  104,  108,  112,  116,  120,  124,  127,  
131,  135,  139,  143,  146,  150,  153,  157,  160,  164,  
167,  171,  174,  177,  180,  183,  186,  189,  192,  195,       //Paste this at top of program
198,  201,  204,  206,  209,  211,  214,  216,  219,  221,  
223,  225,  227,  229,  231,  233,  235,  236,  238,  240,  
241,  243,  244,  245,  246,  247,  248,  249,  250,  251,  
252,  253,  253,  254,  254,  254,  255,  255,  255,  255
};

//============================================
void setup()
{ 
  pinMode(6,OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(4, OUTPUT);

  digitalWrite(6,(gain/1)%2);//LSB
  digitalWrite(5,(gain/2)%2);
  digitalWrite(4,(gain/4)%2);//MSB

  
  pinMode(INPUT_PIN, INPUT);
  pinMode(7, OUTPUT); // this pin will pull the HC-05 pin 34 (key pin) HIGH to switch module to AT mode
  digitalWrite(7, LOW);
  Serial.begin(115200); 
  MyBlue.begin(9600); 
  
}

void loop()
{ 
  adc_value=analogRead(INPUT_PIN);
  samp2=millis();
  if(samp2-samp3>=10){
    samp3=samp2;
    adc_arry[adc_times++]=adc_value;
  }
  
  
  //BT connection
  if (MyBlue.available()){
    Serial.write(MyBlue.read());
  }
  if (Serial.available()){
    MyBlue.write(Serial.read());
    
  }
  
  //=================================
  if(adc_value>ref){
    v3=adc_value;
    if((v3<v2)&&(v2<v1)){  //&&(v1<v0)
        flag=1;
      } 
    v0=v1;
    v1=v2;
    v2=v3;
  }

  if(flag==1&&adc_value>ref){
    bb++;  
  }

  if(adc_value<ref){
    flag=0;
    v0=0;
    v1=0;
    v2=0;
  }

  if(bbo!=bb){
    bbo=bb;
    r1=millis();
  }

  RR=r1-r;
  r=r1;
  
  if(index>=DATA_num){
    index=0;
    //calculate RR
    sort(RR_interval, DATA_num);
    delete_data(RR_interval, DATA_num);
    fill_RR(new_RR, delta_RR, DATA_num-2*DEL);  
  }

  if(HR_times>=5){
    HR_sum=HR_RR[0]+HR_RR[1]+HR_RR[2]+HR_RR[3]+HR_RR[4];
    HR_sum/=5;
    HR_times=0;
  }
  
  if(RR<100&&RR!=0){
    sum+=RR;
  }
  if(RR>100){
    RR+=sum;
    sum=0;  
    RR_interval[index++] = RR;
    HR_RR[HR_times++]=RR;
  }
  if(RR<100){
    RR=0;
  }

  
  //judgment
  float mean  = Mean(new_RR, DATA_num-2*DEL);
  float sdnn  = SDNN(new_RR, DATA_num-2*DEL);
  float cv    = CV(new_RR, DATA_num-2*DEL);
  float rmssd = RMSSD(new_RR, delta_RR, DATA_num-2*DEL);
  //float sdsd  = SDSD(new_RR, delta_RR, DATA_num-2*DEL);

  
  //BT  
  // frequency to data number table
  //  5   0.63Hz
  //  6   0.75Hz
  //  7   0.88Hz
  //  8   1.00Hz
  //  9   1.12Hz
  //  10  1.25Hz
  //  11  1.37Hz
  //  12  1.50Hz
  
  bt_counter=millis();
  if(bt_counter-bt_counter1>=2){
    bt_counter1=bt_counter;
    
    
    MyBlue.print((int)adc_value, DEC);
    MyBlue.print(","); 
    MyBlue.print((int)RR, DEC); 
    MyBlue.print(",");
    MyBlue.print((int)HR_sum, DEC);
    MyBlue.print(","); 
    MyBlue.print((int)sdnn, DEC); 
    MyBlue.print(","); 
    MyBlue.print((int)rmssd, DEC); 
    MyBlue.print(","); 
    MyBlue.print((int)(cv*100), DEC); 
    
  
    for(int i=5;i<=8;i++){
      if(fft_data[i]<10){
        MyBlue.print(",");
        MyBlue.print(fft_data[i], DEC);
      } 
      else MyBlue.print(fft_data[i], DEC);
    }
  
    for(int i=9;i<=12;i++){
      if(fft_data[i]<10){
        MyBlue.print(",");
        MyBlue.print(fft_data[i], DEC);
      } 
      else MyBlue.print(fft_data[i], DEC);
    }
    MyBlue.print(",");
    MyBlue.print((int)alarm, DEC);
    
    
    
    
    MyBlue.println("");
  }
  
  


  //Serial

  
  Serial.print(HR_sum);
  Serial.print(" ");
  Serial.print(sdnn);
  Serial.print(" ");
  Serial.print(cv*100);
  Serial.print(" ");
  Serial.print(rmssd);
  Serial.print(" ");
  //Serial.print(sdsd);
  //Serial.print(" ");

  
  Serial.print(adc_value);
  Serial.print(" ");
  Serial.print(RR);
  Serial.print(" ");
  Serial.print(alarm);
  Serial.print(" ");

  Serial.println("");
  
  //alarm
  if(RR!=0){
    count++;
  }
  if(RR>=1600){
    flag1=1;
  }

  owo3=millis();
  if(owo3-owo4>=5000){
    owo4=owo3;
    adc_times=0;
    decrease=0;
    increase=0;
    up=0;
    down=0;
//    for(int i=0;i<=500;i++){
//      if((adc_arry[i]>=650)||(adc_arry[i]<=100)){
//        up++;
//      }
//      if((adc_arry[i]>=320)&&(adc_arry[i]<=370)){
//        down++;
//      }
//    }
//    if(up>=250){
//      decrease=100;
//      
//    }
//    if(down>=350){
//      increase=110;
//
//    }
//    if(decrease==100){
//      //gain++;
//    }
//    if(increase==110){
//      //gain--;
//    }
//    
//    if(gain>=7){
//      gain=7;
//    }
//    if(gain<=0){
//      gain=0;
//    }
    //======alarm
    
    if((count<=4)||(count>=13)||(flag1==1)){ //||(increase==110)
      alarm=200;
    }

    else{
      alarm=0;
    }
    count=0;
    flag1=0;
    //=====auto gain control
    
    
  }

  //FFT 
  samp0=millis();
  if(samp0-samp1>=250){
    data[owo]=adc_value;
    owo++;
    samp1=samp0;
  }
  
  if(owo>=DATA_num){
    owo=0;  
    FFT(data, DATA_num, SAMPLE_FREQ); 
  }  

  
}


//self-defined func.========================================================================
void fill_RR(float *RR, float *delta_RR, int sz){
    int i;
    for(i=0;i<sz-1;i++){
        delta_RR[i] = RR[i+1]-RR[i]; // 計算RR差值 = RR[i+1]-RR[i]
    }
}

float Mean(float *RR, int sz){
    float sum=0.0;
    int N=sz;
    while(sz>0){
        sum+=RR[sz-1];
        sz--;
    }

    return sum/N;
}

float SDNN(float *RR, int sz){
    float sum=0.0;
    float avg = Mean(RR, sz);
    int N=sz;
    while(sz>0){
        sum+=(RR[sz-1]-avg)*(RR[sz-1]-avg);
        sz--;
    }

    return sqrt(sum/(N-1));
}

float CV(float *RR, int sz){
    return SDNN(RR, sz)/Mean(RR, sz);
}

float RMSSD(float *RR, float *delta_RR, int sz){
    float sum=0.0;
    int i;
    int N=sz;

    fill_RR(RR, delta_RR, sz);
    for(i=0;i<sz-1;i++){
        sum+= delta_RR[i]*delta_RR[i];
    }

    return sqrt(sum/N);
}

/*
float SDSD(float *RR, float *delta_RR, int sz){
    int i;
    float sum=0.0;
    int N=sz;

    for(i=0;i<sz-1;i++){ //*
        sum+= (delta_RR[i]-Mean(delta_RR, N))*(delta_RR[i]-Mean(delta_RR, N));
    }

    return sqrt(sum/(N-1));
}
*/

void sort(float *a, int sz){
    int i,j,tmp;
    for(i=1;i<sz;i++){
        j=i;
        tmp=a[j];
        while(j>0 && a[j-1]>tmp){
            a[j] = a[j-1];
            j--;
        }
        a[j] = tmp;
    }
}

void delete_data(float *RR, int sz){
    int i;
    for(i=DEL;i<sz-DEL;i++) new_RR[i-DEL] = RR[i];
}

float FFT(int in[], int N, float Frequency)
{

unsigned int data[12]={1,2,4,8,16,32,64,128,256,512,1024,2048}; //2^n, n: index
int a,c1,f,o,x;

      a=N;   // 取樣多少點                             
      for(int i=0;i<12;i++)                 //calculating the levels
         { if(data[i]<=a){o=i;} } //o : the degree of 2 (approximation)

int in_ps[data[o]]={0};     //input for sequencing --> data[o] -> 一次抓取的資料點數量 (=N)

// FFT 轉換完出來會是N點的複數( Re + Im )
float out_r[data[o]]={};   //real part of transform
float out_im[data[o]]={};  //imaginory part of transform
           
    x=0;  
    for(int b=0;b<o;b++)                     // bit reversal (Rader's algorithm)
    {
      c1=data[b];//1 2 4 8 .. (2^b)
      f=data[o]/(c1+c1);
          for(int j=0;j<c1;j++)
          { 
            x=x+1;
            in_ps[x]=in_ps[j]+f;
          }
    }


    for(int i=0;i<data[o];i++)            // update input array as per bit reverse order
    {
      if(in_ps[i]<a)
      {out_r[i]=in[in_ps[i]];}
      if(in_ps[i]>a)
      {out_r[i]=in[in_ps[i]-a];}      
    }


int i10,i11,n1;
float e,c,s,tr,ti;

    for(int i=0;i<o;i++)                                    //fft
    {
     i10=data[i];              // overall values of sine/cosine  :
     i11=data[o]/data[i+1];    // loop with similar sine cosine:
     e=360/data[i+1];
     e=0-e;
     n1=0;

        for(int j=0;j<i10;j++)
        {
          c=cosine(e*j);
          s=sine(e*j);    
          n1=j;
          
          for(int k=0;k<i11;k++)
          {
            tr=c*out_r[i10+n1]-s*out_im[i10+n1];
            ti=s*out_r[i10+n1]+c*out_im[i10+n1];

            out_r[n1+i10]=out_r[n1]-tr;
            out_r[n1]=out_r[n1]+tr;

            out_im[n1+i10]=out_im[n1]-ti;
            out_im[n1]=out_im[n1]+ti;          

            n1=n1+i10+i10;
          }       
        }
     }

//---> here onward out_r contains amplitude and our_in conntains frequency (Hz)
  for(int i=0;i<data[o-1];i++)               // getting amplitude from complex number
  {
    out_r[i]=sqrt(out_r[i]*out_r[i]+out_im[i]*out_im[i]); // to  increase the speed delete sqrt (magnitude)
    out_im[i]=i*Frequency/N; // 分到的頻率 --> 取點 ( Fn=(n-1)*Fs/N )

      if(out_im[i]==0);
      else{
        Serial.print(out_im[i]);
        Serial.print("Hz"); // ~ 取樣精度 (x-axis)
        //Serial.print("\t");                            // un comment to print freuency bin    
        Serial.print((int)out_r[i]/100);
        Serial.print("  ");
        fft_data[i]=out_r[i]/100;
        
      }
  }
  //Serial.println("=============");

  // peak detection
  x=0;       
  for(int i=1;i<data[o-1]-1;i++)
  {
    if(out_r[i]>out_r[i-1] && out_r[i]>out_r[i+1]) 
    {
      in_ps[x]=i;    //in_ps array used for storage of peak number
      x=x+1;
    }    
  }

  s=0;
  c=0;
  for(int i=0;i<x;i++)             // re arraange as per magnitude
  {
    for(int j=c;j<x;j++)
    {
        if(out_r[in_ps[i]]<out_r[in_ps[j]]) 
        {
          s=in_ps[i];
          in_ps[i]=in_ps[j];
          in_ps[j]=s;
        }
    }
    c=c+1;
  }

  for(int i=0;i<5;i++)     // updating f_peak array (global variable)with descending order
  {
    f_peaks[i]=out_im[in_ps[i]];
  }
}
    
float sine(int i)
{
  int j=i;
  float out;
  while(j<0){j=j+360;}
  while(j>360){j=j-360;}
  if(j>-1   && j<91){out= sine_data[j];}
  else if(j>90  && j<181){out= sine_data[180-j];}
  else if(j>180 && j<271){out= -sine_data[j-180];}
  else if(j>270 && j<361){out= -sine_data[360-j];}
  return (out/255);
}

float cosine(int i)
{
  int j=i;
  float out;
  while(j<0){j=j+360;}
  while(j>360){j=j-360;}
  if(j>-1   && j<91){out= sine_data[90-j];}
  else if(j>90  && j<181){out= -sine_data[j-90];}
  else if(j>180 && j<271){out= -sine_data[270-j];}
  else if(j>270 && j<361){out= sine_data[j-270];}
  return (out/255);
}
