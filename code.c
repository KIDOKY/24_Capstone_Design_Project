//적외선센서를 이용한 추락방지 스마트 로봇청소기

// 적외선 센서 핀 번호를 정의
#define FRONT_BOTTOM_IR_SENSOR_PIN A1 // 앞밑면
#define LEFT_BOTTOM_IR_SENSOR_PIN A2 //왼밑면
#define RIGHT_BOTTOM_IR_SENSOR_PIN A3 //오른밑면
#define REAR_BOTTOM_IR_SENSOR_PIN A4 // 뒷밑면

// 장애물을 감지할 거리를 정의
#define OBSTACLE_THRESHOLD_FRONT_BOTTOM 300 // 장애물 감지 한계거리
#define OBSTACLE_THRESHOLD_LEFT_BOTTOM 300
#define OBSTACLE_THRESHOLD_RIGHT_BOTTOM 300
#define OBSTACLE_THRESHOLD_REAR_BOTTOM 300
#define NORMAL_MODE  0
#define BLT_MODE  1


const int motor1Pin1 = 3; // A1 전/좌
const int motor1Pin2 = 4; // A2  
const int motor2Pin1 = 5; // B1 전/우
const int motor2Pin2 = 6; // B2
const int motor3Pin1 = 7; // C1 후/좌
const int motor3Pin2 = 8; // C2
const int motor4Pin1 = 9; // D1 후/우
const int motor4Pin2 = 10; // D2
const int switcSerial1 = 11; //스위치버튼
const int SUCTION_MOTOR_PIN1 = 12; // 흡입 모터 핀 정의
const int SUCTION_MOTOR_PIN2 = 13;


bool toggleSwitchState = false; //토글스위치1 (흡입모터, 바퀴4륜)
bool obstacleState = false;
unsigned char  proc_step = 0, op_state = NORMAL_MODE;
unsigned long  current_time = 0, prev_time = 0;
char  blt_data = 0;
bool frontState = false;
bool rearState = false;
bool rightState = false;
bool leftState = false;


void setup() {
    Serial.begin(9600); // 시리얼초기화
    Serial1.begin(9600);
    pinMode(motor1Pin1, OUTPUT);
    pinMode(motor1Pin2, OUTPUT);
    pinMode(motor2Pin1, OUTPUT);
    pinMode(motor2Pin2, OUTPUT);
    pinMode(motor3Pin1, OUTPUT);
    pinMode(motor3Pin2, OUTPUT);
    pinMode(motor4Pin1, OUTPUT);
    pinMode(motor4Pin2, OUTPUT);
    pinMode(switcSerial1, INPUT); //스위치 핀 설정
    pinMode(SUCTION_MOTOR_PIN1, OUTPUT); // 흡입 모터 핀 설정
    pinMode(SUCTION_MOTOR_PIN2, OUTPUT);
}


//4방향 센서 체크 후 낭떠러지 감지시 모터 정지 함수
void checkCliffAndStop() {
    int frontBottomIRSensorValue = analogRead(FRONT_BOTTOM_IR_SENSOR_PIN);
    int leftBottomIRSensorValue = analogRead(LEFT_BOTTOM_IR_SENSOR_PIN);
    int rightBottomIRSensorValue = analogRead(RIGHT_BOTTOM_IR_SENSOR_PIN);
    int rearBottomIRSensorValue = analogRead(REAR_BOTTOM_IR_SENSOR_PIN);


    if (frontBottomIRSensorValue > OBSTACLE_THRESHOLD_FRONT_BOTTOM ||
        leftBottomIRSensorValue > OBSTACLE_THRESHOLD_LEFT_BOTTOM ||
        rightBottomIRSensorValue > OBSTACLE_THRESHOLD_RIGHT_BOTTOM ||
        rearBottomIRSensorValue > OBSTACLE_THRESHOLD_REAR_BOTTOM) {
        stopMotors();
    }
}



//4방향 센서 체크 후 낭떠러지 감지시 참값 반환
bool isObstacleDetected() {
    int frontBottomIRSensorValue = analogRead(FRONT_BOTTOM_IR_SENSOR_PIN);
    int leftBottomIRSensorValue = analogRead(LEFT_BOTTOM_IR_SENSOR_PIN);
    int rightBottomIRSensorValue = analogRead(RIGHT_BOTTOM_IR_SENSOR_PIN);
    int rearBottomIRSensorValue = analogRead(REAR_BOTTOM_IR_SENSOR_PIN);




    if (frontBottomIRSensorValue < 300 ||
        leftBottomIRSensorValue < 300 ||
        rightBottomIRSensorValue < 300 ||
        rearBottomIRSensorValue < 300) {
        return true;
    }
    return false;
}


//전방센서 체크 후 낭떠러지 감지시 참값 반환
bool isFrontDetected() {
    int frontBottomIRSensorValue = analogRead(FRONT_BOTTOM_IR_SENSOR_PIN);


    if (frontBottomIRSensorValue < 300) {
        return true;
    }
    return false;
}


//후방센서 체크 후 낭떠러지 감지시 참값 반환
bool isRearDetected() {
    int rearBottomIRSensorValue = analogRead(REAR_BOTTOM_IR_SENSOR_PIN);


    if (rearBottomIRSensorValue < 300) {
        return true;
    }
    return false;
}


bool isRightDetected() {
    int rightBottomIRSensorValue = analogRead(RIGHT_BOTTOM_IR_SENSOR_PIN);


    if (rightBottomIRSensorValue < 300) {
        return true;
    }
    return false;
}


bool isLeftDetected() {
    int leftBottomIRSensorValue = analogRead(LEFT_BOTTOM_IR_SENSOR_PIN);


    if (leftBottomIRSensorValue < 300) {
        return true;
    }
    return false;
}




void loop()
{
    obstacleState = isObstacleDetected();  // 적외선 센서로 장애물 체크
    frontState = isFrontDetected();
    rearState = isRearDetected();
    rightState = isRightDetected();
    leftState = isLeftDetected();
    toggleSwitchState = digitalRead(11);   //스위치 디지털핀 11번 읽기




    if (toggleSwitchState == LOW)
    {
        stopMotors();
        op_state = BLT_MODE;
    }


    else if (toggleSwitchState == HIGH)
    {
        stopMotors();
        op_state = NORMAL_MODE;
        proc_step = 0;
        prev_time = millis();
    }




    ///////////////////////////////////////////
/* 메뉴얼




   F: 전방 감지&전진
   f: 전진
   B: 후방 감지&후진
   b: 후진
   R: 우회전
   L: 좌회전
   S: DC/흡입 모터 정지
   T: 흡입모터 작동
   P: 흡입모터 정지
   I: 자율주행 감지O
   U: 자율주행 감지X
*/
    if (op_state == BLT_MODE)
    {
        if (Serial1.available())       // 블루투스로부터 데이터가 수신되면
        {
            blt_data = Serial1.read();  // 명령을 읽어옴
        }




        switch (blt_data)
        {
        case 'F':
            if (frontState == true) moveForward();
            else if (frontState == false) stopMotors();
            break;




        case 'f':
            moveForward();
            break;




        case 'B':
            if (rearState == true) moveBackward();
            else if (rearState == false) stopMotors();
            break;




        case 'b':
            moveBackward();
            break;




        case 'R':
            TurnRight();

            break;




        case 'L':
            TurnLeft();

            break;




        case 'S':
            stopMotors();
            stopSuctionMotor();
            break;




        case 'T':
            startSuctionMotor();
            break;




        case 'P':
            stopSuctionMotor();
            break;




        case 'I':  // 자율주행 감지O
            current_time = millis();
            if ((current_time - prev_time) >= 2000)
            {
                prev_time = current_time;
                proc_step++;
                if (proc_step >= 6) proc_step = 0;
            }
            if (proc_step == 0)
            {
                if (frontState == true) {
                    moveForward();
                    startSuctionMotor();
                }
                else if (frontState == false) {
                    stopMotors();
                    stopSuctionMotor();
                }
            }
            else if (proc_step == 1)
            {
                if (leftState == true) {
                    TurnRight();
                    startSuctionMotor();
                }
                else if (leftState == false) {
                    stopMotors();
                    stopSuctionMotor();
                }
            }
            else if (proc_step == 2)
            {
                if (leftState == true) {
                    TurnRight();
                    startSuctionMotor();
                }
                else if (leftState == false) {
                    stopMotors();
                    stopSuctionMotor();
                }
            }
            else if (proc_step == 3)
            {
                if (frontState == true) {
                    moveForward();
                    startSuctionMotor();
                }
                else if (frontState == false) {
                    stopMotors();
                    stopSuctionMotor();
                }

            }
            if (proc_step == 4)
            {
                if (rightState == true) {
                    TurnLeft();
                    startSuctionMotor();
                }
                else if (rightState == false) {
                    stopMotors();
                    stopSuctionMotor();
                }
            }
            else if (proc_step == 5)
            {

                if (rightState == true) {
                    TurnLeft();
                    startSuctionMotor();
                }
                else if (rightState == false) {
                    stopMotors();
                    stopSuctionMotor();
                }
            }


            break;




        case 'U':  // 자율주행 감지X
            current_time = millis();
            if ((current_time - prev_time) >= 2000)
            {
                prev_time = current_time;
                proc_step++;
                if (proc_step >= 6) proc_step = 0;
            }
            if (proc_step == 0)
            {
                moveForward();
                startSuctionMotor();
            }
            else if (proc_step == 1)
            {
                TurnRight();
                startSuctionMotor();
            }
            else if (proc_step == 2)
            {
                TurnRight();
                startSuctionMotor();
            }
            else if (proc_step == 3)
            {
                moveForward();
                startSuctionMotor();
            }
            else if (proc_step == 4)
            {
                TurnLeft();
                startSuctionMotor();
            }
            else if (proc_step == 5)
            {
                TurnLeft();
                startSuctionMotor();
            }

            break;


        case 'K':  // 자율주행 감지X
            current_time = millis();
            if ((current_time - prev_time) >= 3000)
            {
                prev_time = current_time;
                proc_step++;
                if (proc_step >= 3) proc_step = 0;
            }
            if (proc_step == 0)
            {
                moveForward();
            }

            else if (proc_step == 2)
            {
                moveBackward();
            }

            break;




        default:
            // 지정되지 않은 문자에 대해서는 아무 작업도 하지 않음
            break;
        }
    }
} //loop문 종료


// 모터 정지 함수
void stopMotors() {
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, LOW);
    digitalWrite(motor2Pin1, LOW);
    digitalWrite(motor2Pin2, LOW);
    digitalWrite(motor3Pin1, LOW);
    digitalWrite(motor3Pin2, LOW);
    digitalWrite(motor4Pin1, LOW);
    digitalWrite(motor4Pin2, LOW);
}


void moveForward() {
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, HIGH);
    analogWrite(motor1Pin2, 100);
    digitalWrite(motor2Pin1, HIGH);
    digitalWrite(motor2Pin2, LOW);
    analogWrite(motor2Pin1, 100);
    digitalWrite(motor3Pin1, LOW);
    digitalWrite(motor3Pin2, HIGH);
    analogWrite(motor3Pin2, 100);
    digitalWrite(motor4Pin1, HIGH);
    digitalWrite(motor4Pin2, LOW);
    analogWrite(motor4Pin1, 100);
}


void moveLeft() {
    digitalWrite(motor1Pin1, HIGH);
    digitalWrite(motor1Pin2, LOW);
    analogWrite(motor1Pin1, 100);
    digitalWrite(motor2Pin1, HIGH);
    digitalWrite(motor2Pin2, LOW);
    analogWrite(motor2Pin1, 100);
    digitalWrite(motor3Pin1, LOW);
    digitalWrite(motor3Pin2, HIGH);
    analogWrite(motor3Pin2, 100);
    digitalWrite(motor4Pin1, LOW);
    digitalWrite(motor4Pin2, HIGH);
    analogWrite(motor4Pin2, 100);
}


void moveRight() {
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, HIGH);
    analogWrite(motor1Pin2, 160);
    digitalWrite(motor2Pin1, LOW);
    digitalWrite(motor2Pin2, HIGH);
    analogWrite(motor2Pin2, 160);
    digitalWrite(motor3Pin1, HIGH);
    digitalWrite(motor3Pin2, LOW);
    analogWrite(motor3Pin1, 160);
    digitalWrite(motor4Pin1, HIGH);
    digitalWrite(motor4Pin2, LOW);
    analogWrite(motor4Pin1, 160);
}


void moveBackward() {
    digitalWrite(motor1Pin1, HIGH);
    digitalWrite(motor1Pin2, LOW);
    analogWrite(motor1Pin1, 100);
    digitalWrite(motor2Pin1, LOW);
    digitalWrite(motor2Pin2, HIGH);
    analogWrite(motor2Pin2, 100);
    digitalWrite(motor3Pin1, HIGH);
    digitalWrite(motor3Pin2, LOW);
    analogWrite(motor3Pin1, 100);
    digitalWrite(motor4Pin1, LOW);
    digitalWrite(motor4Pin2, HIGH);
    analogWrite(motor4Pin2, 100);
}


// 제자리 시계 방향 회전
void TurnRight() {
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, HIGH);
    analogWrite(motor1Pin2, 160);
    digitalWrite(motor2Pin1, LOW);
    digitalWrite(motor2Pin2, HIGH);
    analogWrite(motor2Pin2, 160);
    digitalWrite(motor3Pin1, LOW);
    digitalWrite(motor3Pin2, HIGH);
    analogWrite(motor3Pin2, 160);
    digitalWrite(motor4Pin1, LOW);
    digitalWrite(motor4Pin2, HIGH);
    analogWrite(motor4Pin2, 160);
}


// 제자리 반시계 방향 회전
void TurnLeft() {
    digitalWrite(motor1Pin1, HIGH);
    digitalWrite(motor1Pin2, LOW);
    analogWrite(motor1Pin1, 160);
    digitalWrite(motor2Pin1, HIGH);
    digitalWrite(motor2Pin2, LOW);
    analogWrite(motor2Pin1, 180);
    digitalWrite(motor3Pin1, HIGH);
    digitalWrite(motor3Pin2, LOW);
    analogWrite(motor3Pin1, 160);
    digitalWrite(motor4Pin1, HIGH);
    digitalWrite(motor4Pin2, LOW);
    analogWrite(motor4Pin1, 180);
}


// 흡입 모터 제어 함수
void startSuctionMotor() {
    digitalWrite(SUCTION_MOTOR_PIN1, HIGH);
    digitalWrite(SUCTION_MOTOR_PIN2, LOW);
} //오류로 인해 HIGH를 줘야 멈춤


void stopSuctionMotor() {
    digitalWrite(SUCTION_MOTOR_PIN1, LOW);
    digitalWrite(SUCTION_MOTOR_PIN2, LOW);
} //오류로 인해 LOW를 줘야 작동함