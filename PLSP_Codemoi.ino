#include <ESP32Servo.h>
#include <GY33.h>
#include <stdlib.h>

// ================= CHÂN KẾT NỐI =================
#define MOTOR_PIN       5
#define OBSTACLE_PIN    32

#define SG1_PIN         18
#define SG2_PIN         19
#define SG3_PIN         25
#define SG4_PIN         26

// Motor kích hoạt mức thấp
#define MOTOR_ON        LOW
#define MOTOR_OFF       HIGH

// Đổi thành HIGH nếu cảm biến báo HIGH khi có vật
#define OBSTACLE_ACTIVE LOW

// ================= THỜI GIAN =================

// Mỗi bước servo di chuyển 1 độ.
// Tăng giá trị này để chạy chậm hơn.
const unsigned long THOI_GIAN_BUOC = 12;

// Chờ cơ cấu ổn định sau mỗi thao tác.
const unsigned long THOI_GIAN_ON_DINH = 100;

// Chờ vật ổn định sau khi dừng băng tải.
const unsigned long SETTLE_TIME = 300;

// Thời gian tối đa chờ nhận diện màu.
const unsigned long COLOR_TIMEOUT = 2000;

// Khoảng cách giữa các lần đọc màu.
const unsigned long READ_INTERVAL = 100;

// ================= THIẾT BỊ =================
GY33_I2C sensor;

Servo sg1;
Servo sg2;
Servo sg3;
Servo sg4;

Servo* danhSachServo[] = {
  &sg1, &sg2, &sg3, &sg4
};

// Thứ tự SG1, SG2, SG3, SG4.
// Đây là góc điều khiển, không phải góc đo thực tế.
int gocHienTai[] = {87, 180, 90, 120};
int gocMucTieu[] = {87, 180, 90, 120};

unsigned long lanDiChuyenCuoi = 0;

// ================= MÀU SẮC =================
enum ObjectColor {
  COLOR_UNKNOWN,
  COLOR_RED,
  COLOR_GREEN,
  COLOR_BLUE
};

// ================= TRẠNG THÁI =================
enum SystemState {
  RUNNING,
  WAIT_SETTLE,
  READ_COLOR,
  WAIT_OBJECT_CLEAR
};

SystemState state = RUNNING;

bool cheDoTuDong = true;

unsigned long stoppedAt = 0;
unsigned long readStartedAt = 0;
unsigned long lastReadAt = 0;

String lenhSerial = "";

// Khai báo trước các hàm dùng kiểu ObjectColor,
// tránh lỗi tạo prototype tự động của Arduino.
ObjectColor nhanDienMau(int r, int g, int b);
void xuLyVat(ObjectColor color);

// ================= ĐIỀU KHIỂN BĂNG TẢI =================

void chayBangTai() {
  digitalWrite(MOTOR_PIN, MOTOR_ON);
}

void dungBangTai() {
  digitalWrite(MOTOR_PIN, MOTOR_OFF);
}

bool coVat() {
  return digitalRead(OBSTACLE_PIN) == OBSTACLE_ACTIVE;
}

// ================= CHUYỂN ĐỘNG SERVO =================

// Cập nhật chuyển động từ từ cho cả 4 servo.
void capNhatServo() {
  unsigned long bayGio = millis();

  if (bayGio - lanDiChuyenCuoi < THOI_GIAN_BUOC) {
    return;
  }

  lanDiChuyenCuoi = bayGio;

  for (int i = 0; i < 4; i++) {
    if (gocHienTai[i] == gocMucTieu[i]) {
      continue;
    }

    if (gocHienTai[i] < gocMucTieu[i]) {
      gocHienTai[i]++;
    } else {
      gocHienTai[i]--;
    }

    danhSachServo[i]->write(gocHienTai[i]);

    if (gocHienTai[i] == gocMucTieu[i]) {
      Serial.print("sg");
      Serial.print(i + 1);
      Serial.print(" da den goc dieu khien: ");
      Serial.println(gocHienTai[i]);
    }
  }
}

bool servoDangDiChuyen() {
  for (int i = 0; i < 4; i++) {
    if (gocHienTai[i] != gocMucTieu[i]) {
      return true;
    }
  }

  return false;
}

void choServoHoanTat() {
  while (servoDangDiChuyen()) {
    capNhatServo();
    delay(1);
  }
}

// Di chuyển một servo rồi chờ hoàn tất.
void diChuyenCham(int soServo, int goc) {
  if (soServo < 1 || soServo > 4 ||
      goc < 0 || goc > 180) {
    return;
  }

  choServoHoanTat();

  gocMucTieu[soServo - 1] = goc;

  choServoHoanTat();
  delay(THOI_GIAN_ON_DINH);
}

// SG2 và SG4 cùng bắt đầu di chuyển.
// Servo có quãng đường ngắn hơn sẽ đến đích trước.
void diChuyenSG2SG4(int gocSG2, int gocSG4) {
  if (gocSG2 < 0 || gocSG2 > 180 ||
      gocSG4 < 0 || gocSG4 > 180) {
    return;
  }

  choServoHoanTat();

  gocMucTieu[1] = gocSG2;
  gocMucTieu[3] = gocSG4;

  choServoHoanTat();
  delay(THOI_GIAN_ON_DINH);
}

// ================= 5 THAO TÁC SERVO =================

// L2: Gắp và nâng vật.
void gapVat() {
  Serial.println("Bat dau gap vat...");

  // SG1 đến vị trí gắp.
  diChuyenCham(1, 87);

  // SG2 và SG4 cùng tiếp cận vật.
  diChuyenSG2SG4(105, 45);

  // SG3 đóng ngón tay.
  diChuyenCham(3, 30);

  // SG2 và SG4 cùng nâng vật.
  diChuyenSG2SG4(180, 120);

  Serial.println("Hoan tat gap va nang vat.");
}

// L3: Thả vật vào hộp đỏ.
void phanLoaiDo() {
  Serial.println("Dua vat vao hop DO...");

  diChuyenCham(1, 27);
  diChuyenSG2SG4(125, 75);

  // Mở ngón tay để thả vật.
  diChuyenCham(3, 90);

  // Nâng tay lên.
  diChuyenSG2SG4(180, 120);

  Serial.println("Hoan tat thao tac hop DO.");
}

// L5: Thả vật vào hộp xanh lá.
void phanLoaiXanhLa() {
  Serial.println("Dua vat vao hop XANH LA...");

  diChuyenCham(1, 12);
  diChuyenSG2SG4(145, 100);

  diChuyenCham(3, 90);

  diChuyenSG2SG4(180, 120);

  Serial.println("Hoan tat thao tac hop XANH LA.");
}

// L4: Thả vật vào hộp xanh dương.
void phanLoaiXanhDuong() {
  Serial.println("Dua vat vao hop XANH DUONG...");

  diChuyenCham(1, 0);
  diChuyenSG2SG4(155, 120);

  diChuyenCham(3, 90);

  diChuyenSG2SG4(180, 120);

  Serial.println("Hoan tat thao tac hop XANH DUONG.");
}

// L1: Về vị trí ban đầu.
void veViTriBanDau() {
  Serial.println("Quay ve vi tri ban dau...");

  // Giữ nguyên thứ tự SG4 -> SG2 -> SG1 -> SG3.
  diChuyenCham(4, 120);
  diChuyenCham(2, 180);
  diChuyenCham(1, 87);
  diChuyenCham(3, 90);

  Serial.println("Da ve vi tri ban dau.");
}

// ================= NHẬN DIỆN MÀU =================

// Nhận diện tạm thời bằng kênh RGB lớn nhất.
// Cần hiệu chỉnh theo số đo thực tế của GY33.
ObjectColor nhanDienMau(int r, int g, int b) {
  if (r > g && r > b) {
    return COLOR_RED;
  }

  if (g > r && g > b) {
    return COLOR_GREEN;
  }

  if (b > r && b > g) {
    return COLOR_BLUE;
  }

  return COLOR_UNKNOWN;
}

void xuLyVat(ObjectColor color) {
  if (color == COLOR_UNKNOWN) {
    return;
  }

  // Băng tải dừng trong toàn bộ quá trình gắp.
  dungBangTai();

  gapVat();

  switch (color) {
    case COLOR_RED:
      phanLoaiDo();
      break;

    case COLOR_GREEN:
      phanLoaiXanhLa();
      break;

    case COLOR_BLUE:
      phanLoaiXanhDuong();
      break;

    default:
      break;
  }

  veViTriBanDau();

  Serial.println("Hoan tat chu trinh phan loai.");
}

// ================= LỆNH SERIAL =================

void vaoCheDoThuCong() {
  cheDoTuDong = false;
  dungBangTai();
}

// Xử lý lệnh Serial.
void xuLyLenh(String lenh) {
  lenh.trim();
  lenh.toLowerCase();

  if (lenh.length() == 0) {
    return;
  }

  if (lenh == "stop") {
    vaoCheDoThuCong();

    Serial.println("Da dung bang tai. Che do thu cong.");
    return;
  }

  if (lenh == "auto") {
    // Chờ hoàn tất lệnh góc thủ công trước khi chạy.
    choServoHoanTat();

    dungBangTai();
    veViTriBanDau();

    cheDoTuDong = true;
    state = RUNNING;

    // Nếu đang có vật, giữ băng tải dừng để đọc màu.
    if (coVat()) {
      stoppedAt = millis();
      state = WAIT_SETTLE;
    } else {
      chayBangTai();
    }

    Serial.println("Da bat che do TU DONG.");
    return;
  }

  if (lenh == "l1" || lenh == "home") {
    vaoCheDoThuCong();
    veViTriBanDau();
    return;
  }

  if (lenh == "l2" || lenh == "gap") {
    vaoCheDoThuCong();
    gapVat();
    return;
  }

  if (lenh == "l3" || lenh == "hopdo") {
    vaoCheDoThuCong();
    phanLoaiDo();
    return;
  }

  if (lenh == "l4") {
    vaoCheDoThuCong();
    phanLoaiXanhDuong();
    return;
  }

  if (lenh == "l5") {
    vaoCheDoThuCong();
    phanLoaiXanhLa();
    return;
  }

  // Chu trình thủ công hoàn chỉnh theo từng màu.
  if (lenh == "do") {
    vaoCheDoThuCong();
    xuLyVat(COLOR_RED);
    return;
  }

  if (lenh == "xanhla" || lenh == "green") {
    vaoCheDoThuCong();
    xuLyVat(COLOR_GREEN);
    return;
  }

  if (lenh == "xanhduong" || lenh == "blue") {
    vaoCheDoThuCong();
    xuLyVat(COLOR_BLUE);
    return;
  }

  // Điều khiển từng servo: sg1-90, sg2-125...
  int viTriGach = lenh.indexOf('-');

  if (viTriGach < 0) {
    Serial.println(
      "Sai lenh! Dung auto, stop, L1-L5 hoac sg1-90."
    );
    return;
  }

  String tenServo = lenh.substring(0, viTriGach);
  String chuoiGoc = lenh.substring(viTriGach + 1);

  tenServo.trim();
  chuoiGoc.trim();

  int chiSoServo = -1;

  if (tenServo == "sg1") {
    chiSoServo = 0;
  } else if (tenServo == "sg2") {
    chiSoServo = 1;
  } else if (tenServo == "sg3") {
    chiSoServo = 2;
  } else if (tenServo == "sg4") {
    chiSoServo = 3;
  } else {
    Serial.println("Ten servo phai la sg1, sg2, sg3, sg4.");
    return;
  }

  if (chuoiGoc.length() == 0 || chuoiGoc.length() > 3) {
    Serial.println("Nhap goc tu 0 den 180. Vi du: sg1-90");
    return;
  }

  for (unsigned int i = 0; i < chuoiGoc.length(); i++) {
    if (chuoiGoc[i] < '0' || chuoiGoc[i] > '9') {
      Serial.println("Goc phai la so nguyen tu 0 den 180.");
      return;
    }
  }

  char* kyTuDu;
  long goc = strtol(chuoiGoc.c_str(), &kyTuDu, 10);

  if (*kyTuDu != '\0' || goc < 0 || goc > 180) {
    Serial.println("Goc khong hop le! Nhap tu 0 den 180.");
    return;
  }

  vaoCheDoThuCong();

  // loop() cập nhật chuyển động từ từ.
  gocMucTieu[chiSoServo] = (int)goc;

  Serial.print(tenServo);
  Serial.print(" di chuyen den goc: ");
  Serial.println(goc);
}

void docLenhSerial() {
  while (Serial.available() > 0) {
    char kyTu = Serial.read();

    if (kyTu == '\n' || kyTu == '\r') {
      if (lenhSerial.length() > 0) {
        String lenh = lenhSerial;
        lenhSerial = "";

        xuLyLenh(lenh);
      }
    } else {
      lenhSerial += kyTu;
    }

    capNhatServo();
  }
}

// ================= CHẠY TỰ ĐỘNG =================

void capNhatTuDong() {
  unsigned long now = millis();

  switch (state) {
    case RUNNING:
      if (coVat()) {
        dungBangTai();

        stoppedAt = now;
        state = WAIT_SETTLE;

        Serial.println("Phat hien vat. Dung bang tai.");
      }
      break;

    case WAIT_SETTLE:
      if (now - stoppedAt >= SETTLE_TIME) {
        readStartedAt = now;
        lastReadAt = now - READ_INTERVAL;
        state = READ_COLOR;

        Serial.println("Bat dau doc mau...");
      }
      break;

    case READ_COLOR:
      if (now - readStartedAt >= COLOR_TIMEOUT) {
        Serial.println(
          "Khong nhan dien duoc mau. Cho vat di qua."
        );

        chayBangTai();
        state = WAIT_OBJECT_CLEAR;
        break;
      }

      if (now - lastReadAt >= READ_INTERVAL) {
        lastReadAt = now;

        if (sensor.update()) {
          GY33_Processed rgb = sensor.getProcessed();

          Serial.print("R: ");
          Serial.print(rgb.r);
          Serial.print(" | G: ");
          Serial.print(rgb.g);
          Serial.print(" | B: ");
          Serial.println(rgb.b);

          ObjectColor color = nhanDienMau(
            rgb.r, rgb.g, rgb.b
          );

          if (color != COLOR_UNKNOWN) {
            xuLyVat(color);

            chayBangTai();
            state = WAIT_OBJECT_CLEAR;

            Serial.println("Bang tai chay tiep.");
          }
        }
      }
      break;

    case WAIT_OBJECT_CLEAR:
      // Chờ cảm biến hết báo vật để tránh xử lý lặp.
      if (!coVat()) {
        state = RUNNING;

        Serial.println("San sang phat hien vat tiep theo.");
      }
      break;
  }
}

// ================= KHỞI TẠO =================

void setup() {
  Serial.begin(115200);

  // Đặt mức tắt trước khi chuyển sang OUTPUT.
  digitalWrite(MOTOR_PIN, MOTOR_OFF);
  pinMode(MOTOR_PIN, OUTPUT);

  pinMode(OBSTACLE_PIN, INPUT_PULLUP);

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  sg1.setPeriodHertz(50);
  sg2.setPeriodHertz(50);
  sg3.setPeriodHertz(50);
  sg4.setPeriodHertz(50);

  sg1.attach(SG1_PIN);
  sg2.attach(SG2_PIN);
  sg3.attach(SG3_PIN);
  sg4.attach(SG4_PIN);

  // Gửi góc khởi tạo khi bật nguồn.
  for (int i = 0; i < 4; i++) {
    danhSachServo[i]->write(gocHienTai[i]);
  }

  // Giữ API khởi tạo GY33 từ code bạn cung cấp.
  sensor.begin();

  delay(500);

  Serial.println("=== HE THONG PHAN LOAI MAU ===");
  Serial.println("auto : Bat che do tu dong");
  Serial.println("stop : Dung bang tai, chuyen sang thu cong");
  Serial.println("L1   : Ve vi tri ban dau");
  Serial.println("L2   : Gap va nang vat");
  Serial.println("L3   : Tha vao hop DO");
  Serial.println("L4   : Tha vao hop XANH DUONG");
  Serial.println("L5   : Tha vao hop XANH LA");
  Serial.println("do / xanhla / xanhduong : Chu trinh theo mau");
  Serial.println("Lenh cu: gap, hopdo, home");
  Serial.println("Dieu khien rieng: sg1-90, sg2-125...");
  Serial.println("Moi lenh mot dong, roi nhan Enter.");
  Serial.println("He thong khoi dong o che do TU DONG.");

  if (coVat()) {
    dungBangTai();
    stoppedAt = millis();
    state = WAIT_SETTLE;
  } else {
    state = RUNNING;
    chayBangTai();
  }
}

// ================= VÒNG LẶP =================

void loop() {
  docLenhSerial();

  capNhatServo();

  if (cheDoTuDong) {
    capNhatTuDong();
  }
}