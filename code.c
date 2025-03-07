//���ܼ������� �̿��� �߶����� ����Ʈ �κ�û�ұ�

// ���ܼ� ���� �� ��ȣ�� ����
#define FRONT_BOTTOM_IR_SENSOR_PIN A1 // �չظ�
#define LEFT_BOTTOM_IR_SENSOR_PIN A2 //�޹ظ�
#define RIGHT_BOTTOM_IR_SENSOR_PIN A3 //�����ظ�
#define REAR_BOTTOM_IR_SENSOR_PIN A4 // �޹ظ�

// ��ֹ��� ������ �Ÿ��� ����
#define OBSTACLE_THRESHOLD_FRONT_BOTTOM 300 // ��ֹ� ���� �Ѱ�Ÿ�
#define OBSTACLE_THRESHOLD_LEFT_BOTTOM 300
#define OBSTACLE_THRESHOLD_RIGHT_BOTTOM 300
#define OBSTACLE_THRESHOLD_REAR_BOTTOM 300
#define NORMAL_MODE  0
#define BLT_MODE  1


const int motor1Pin1 = 3; // A1 ��/��
const int motor1Pin2 = 4; // A2  
const int motor2Pin1 = 5; // B1 ��/��
const int motor2Pin2 = 6; // B2
const int motor3Pin1 = 7; // C1 ��/��
const int motor3Pin2 = 8; // C2
const int motor4Pin1 = 9; // D1 ��/��
const int motor4Pin2 = 10; // D2
const int switcSerial1 = 11; //����ġ��ư
const int SUCTION_MOTOR_PIN1 = 12; // ���� ���� �� ����
const int SUCTION_MOTOR_PIN2 = 13;


bool toggleSwitchState = false; //��۽���ġ1 (���Ը���, ����4��)
bool obstacleState = false;
unsigned char  proc_step = 0, op_state = NORMAL_MODE;
unsigned long  current_time = 0, prev_time = 0;
char  blt_data = 0;
bool frontState = false;
bool rearState = false;
bool rightState = false;
bool leftState = false;


void setup() {
    Serial.begin(9600); // �ø����ʱ�ȭ
    Serial1.begin(9600);
    pinMode(motor1Pin1, OUTPUT);
    pinMode(motor1Pin2, OUTPUT);
    pinMode(motor2Pin1, OUTPUT);
    pinMode(motor2Pin2, OUTPUT);
    pinMode(motor3Pin1, OUTPUT);
    pinMode(motor3Pin2, OUTPUT);
    pinMode(motor4Pin1, OUTPUT);
    pinMode(motor4Pin2, OUTPUT);
    pinMode(switcSerial1, INPUT); //����ġ �� ����
    pinMode(SUCTION_MOTOR_PIN1, OUTPUT); // ���� ���� �� ����
    pinMode(SUCTION_MOTOR_PIN2, OUTPUT);
}


//4���� ���� üũ �� �������� ������ ���� ���� �Լ�
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



//4���� ���� üũ �� �������� ������ ���� ��ȯ
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


//���漾�� üũ �� �������� ������ ���� ��ȯ
bool isFrontDetected() {
    int frontBottomIRSensorValue = analogRead(FRONT_BOTTOM_IR_SENSOR_PIN);


    if (frontBottomIRSensorValue < 300) {
        return true;
    }
    return false;
}


//�Ĺ漾�� üũ �� �������� ������ ���� ��ȯ
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
    obstacleState = isObstacleDetected();  // ���ܼ� ������ ��ֹ� üũ
    frontState = isFrontDetected();
    rearState = isRearDetected();
    rightState = isRightDetected();
    leftState = isLeftDetected();
    toggleSwitchState = digitalRead(11);   //����ġ �������� 11�� �б�




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
/* �޴���




   F: ���� ����&����
   f: ����
   B: �Ĺ� ����&����
   b: ����
   R: ��ȸ��
   L: ��ȸ��
   S: DC/���� ���� ����
   T: ���Ը��� �۵�
   P: ���Ը��� ����
   I: �������� ����O
   U: �������� ����X
*/
    if (op_state == BLT_MODE)
    {
        if (Serial1.available())       // ��������κ��� �����Ͱ� ���ŵǸ�
        {
            blt_data = Serial1.read();  // ����� �о��
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




        case 'I':  // �������� ����O
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




        case 'U':  // �������� ����X
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


        case 'K':  // �������� ����X
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
            // �������� ���� ���ڿ� ���ؼ��� �ƹ� �۾��� ���� ����
            break;
        }
    }
} //loop�� ����


// ���� ���� �Լ�
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


// ���ڸ� �ð� ���� ȸ��
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


// ���ڸ� �ݽð� ���� ȸ��
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


// ���� ���� ���� �Լ�
void startSuctionMotor() {
    digitalWrite(SUCTION_MOTOR_PIN1, HIGH);
    digitalWrite(SUCTION_MOTOR_PIN2, LOW);
} //������ ���� HIGH�� ��� ����


void stopSuctionMotor() {
    digitalWrite(SUCTION_MOTOR_PIN1, LOW);
    digitalWrite(SUCTION_MOTOR_PIN2, LOW);
} //������ ���� LOW�� ��� �۵���