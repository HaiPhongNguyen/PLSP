/*
  ESP32 + TCS34725 - NHAN DIEN TRUOC, VAN CHUYEN SAU
  1. Motor OFF, dat tung vat truoc cam bien mau.
  2. 3 lan doc lien tiep cung mau -> luu mau -> motor ON.
  3. Cam bien vat can o vi tri gap bao vat -> motor OFF.
  4. Cho 300 ms -> gap -> phan loai theo mau da luu -> ve home.
  5. Cho cam bien gap het bao vat 300 ms -> lap lai tu buoc 1.

  Vi tri doc mau phai nam TRUOC cam bien vat can theo chieu bang tai.
  Chi dat 1 vat moi chu trinh; cho thong bao San sang roi dat vat tiep.
  Nen khong vat can co ty le RGB gan can bang. Nguong mau can can chinh
  theo Serial Monitor; phan mem khong the chac chan co vat chi tu RGB.
  Giu chan, muc relay va cac goc servo cua code goc.
  Cac thao tac servo van cho dong bo nhu ban goc: lenh Serial stop
  chi duoc xu ly sau thao tac servo dang chay, khong phai dung khan cap.
  Thu vien: ESP32Servo, Adafruit TCS34725 (kem Adafruit BusIO).
  Serial Monitor: 115200 baud, Newline.
*/
#include <ESP32Servo.h>
#include <Wire.h>
#include <Adafruit_TCS34725.h>
#include <stdlib.h>

// ================= CHÂN KẾT NỐI =================

#define SDA_PIN         21
#define SCL_PIN         22

#define MOTOR_PIN       5
#define OBSTACLE_PIN    32

#define SG1_PIN         18
#define SG2_PIN         19
#define SG3_PIN         25
#define SG4_PIN         26

// Motor kích hoạt mức thấp.
#define MOTOR_ON        LOW
#define MOTOR_OFF       HIGH

// Đổi thành HIGH nếu cảm biến báo HIGH khi có vật.
#define OBSTACLE_ACTIVE LOW

// ================= THỜI GIAN =================

// Servo di chuyển mỗi bước 1 độ.
// Tăng giá trị này để servo chạy chậm hơn.
const unsigned long THOI_GIAN_BUOC = 10;

// Chờ cơ cấu ổn định sau thao tác.
const unsigned long THOI_GIAN_ON_DINH = 200;

// Chờ vật ổn định sau khi dừng băng tải.
const unsigned long SETTLE_TIME = 300;

// Quá thời gian này mà vật chưa đến vị trí gắp: dừng và báo lỗi.
// Chỉnh theo chiều dài / tốc độ băng tải thực tế.
const unsigned long TRANSPORT_TIMEOUT = 15000;
const unsigned long CLEAR_STABLE_TIME = 300;

// Khoảng cách giữa các lần đọc màu.
const unsigned long READ_INTERVAL = 100;

// ================= THIẾT BỊ =================

Adafruit_TCS34725 sensor(
  TCS34725_INTEGRATIONTIME_50MS,
  TCS34725_GAIN_4X
);

Servo sg1;
Servo sg2;
Servo sg3;
Servo sg4;

Servo* danhSachServo[] = {
  &sg1, &sg2, &sg3, &sg4
};

// Thứ tự SG1, SG2, SG3, SG4.
// Đây là góc điều khiển, không phải góc đo thực tế.
int gocHienTai[] = {87, 180, 110, 120};
int gocMucTieu[] = {87, 180, 110, 120};

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
  WAIT_COLOR,
  TRANSPORT,
  WAIT_SETTLE,
  WAIT_OBJECT_CLEAR,
  FAULT
};

SystemState state = WAIT_OBJECT_CLEAR;
ObjectColor mauDaChot = COLOR_UNKNOWN;
unsigned long transportStartedAt = 0;
unsigned long clearStartedAt = 0;
bool dangDemHetVat = false;

ObjectColor mauTruoc = COLOR_UNKNOWN;
uint8_t soLanCungMau = 0;

// Cần 3 lần liên tiếp nhận cùng màu.
const uint8_t SO_LAN_XAC_NHAN = 3;

// Chênh lệch tối thiểu giữa tỷ lệ cao nhất và cao thứ hai.
// Đơn vị: điểm phần trăm. Cần chỉnh theo vật màu thực tế.
// Ví dụ: G=35%, R=33%, B=32% -> không xác định.
const float CHENH_LECH_MAU_MIN = 10.0f;
// Ngưỡng thử ban đầu, cần chỉnh theo số đo thực tế.
const uint16_t CLEAR_MIN = 20;
// 50 ms: loại số đo gần bão hòa của TCS34725.
const uint16_t RAW_MAX = 20000;

bool cheDoTuDong = true;

unsigned long stoppedAt = 0;
unsigned long lastReadAt = 0;

String lenhSerial = "";

// Khai báo trước các hàm sử dụng kiểu ObjectColor.
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

  // Bước 1: SG1 đến vị trí gắp.
  diChuyenCham(1, 87);

  // Bước 2: SG2 và SG4 cùng tiếp cận vật.
  diChuyenSG2SG4(102, 45);

  diChuyenCham(3, 15);

  diChuyenSG2SG4(160, 75);
  // Bước 3: SG3 co ngón tay gắp vật.
  

  // Bước 4: SG2 và SG4 cùng nâng vật.
  diChuyenSG2SG4(180, 120);
  Serial.println("Hoan tat gap va nang vat.");
}

// L3: Thả vật vào hộp đỏ.
void phanLoaiDo() {
  Serial.println("Dua vat vao hop DO...");

  diChuyenCham(1, 25);
  diChuyenSG2SG4(140, 90);

  // Mở ngón tay.
  diChuyenCham(3, 90);

  // Nâng tay.
  diChuyenSG2SG4(180, 120);

  Serial.println("Hoan tat thao tac hop DO.");
}

// L5: Thả vật vào hộp xanh lá.
void phanLoaiXanhLa() {
  Serial.println("Dua vat vao hop XANH LA...");

  diChuyenCham(1, 12);
  diChuyenSG2SG4(150, 110);

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

  // Giữ thứ tự chuyển động thực tế của code gốc.
    diChuyenCham(1, 87);
  diChuyenCham(4, 120);
  diChuyenCham(2, 180);

  diChuyenCham(3, 110);

  Serial.println("Da ve vi tri ban dau.");
}

// ================= NHẬN DIỆN MÀU =================

// Tỷ lệ màu = kênh / (R + G + B) * 100.
// Chọn tỷ lệ cao nhất nếu hơn tỷ lệ thứ hai đủ rõ.
ObjectColor nhanDienMau(int r, int g, int b) {
  uint32_t tong = (uint32_t)r + g + b;

  if (tong == 0) {
    return COLOR_UNKNOWN;
  }

  float tyLe[3] = {
    100.0f * r / tong,
    100.0f * g / tong,
    100.0f * b / tong
  };

  int lonNhat = 0;

  for (int i = 1; i < 3; i++) {
    if (tyLe[i] > tyLe[lonNhat]) {
      lonNhat = i;
    }
  }

  float thuHai = -1.0f;

  for (int i = 0; i < 3; i++) {
    if (i != lonNhat && tyLe[i] > thuHai) {
      thuHai = tyLe[i];
    }
  }

  // Loại trường hợp bằng nhau hoặc chênh lệch quá nhỏ.
  if (tyLe[lonNhat] <= thuHai ||
      tyLe[lonNhat] - thuHai < CHENH_LECH_MAU_MIN) {
    return COLOR_UNKNOWN;
  }

  if (lonNhat == 0) {
    return COLOR_RED;
  }

  if (lonNhat == 1) {
    return COLOR_GREEN;
  }

  return COLOR_BLUE;
}

// Chỉ reset dữ liệu màu, không làm servo di chuyển.
void resetNhanDien() {
  mauTruoc = COLOR_UNKNOWN;
  soLanCungMau = 0;
  mauDaChot = COLOR_UNKNOWN;
}

// Luôn dừng motor khi quay lại chờ. Cảm biến gắp phải hết báo vật
// ổn định trước khi cho phép nhận diện vật tiếp theo.
void batDauChoVat() {
  dungBangTai();
  resetNhanDien();
  dangDemHetVat = false;
  state = WAIT_OBJECT_CLEAR;
}

void baoLoiVanChuyen() {
  dungBangTai();
  resetNhanDien();
  state = FAULT;
  Serial.println("LOI: Vat khong toi cam bien trong 15 giay.");
  Serial.println("Kiem tra / lay vat ra khoi bang tai, gui auto de thu lai.");
}

void xuLyVat(ObjectColor color) {
  if (color == COLOR_UNKNOWN) {
    return;
  }

  // Băng tải dừng suốt quá trình gắp/thả.
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
  batDauChoVat();
}

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
    // Chờ hoàn tất lệnh góc thủ công.
    choServoHoanTat();

    dungBangTai();
    veViTriBanDau();

    cheDoTuDong = true;
    batDauChoVat();
    Serial.println("TU DONG: motor dung, cho dat vat truoc cam bien mau.");
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

  // Chu trình thủ công theo màu.
  // Các lệnh này chạy trực tiếp, không chờ cảm biến vật cản.
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

  // Điều khiển riêng: sg1-90, sg2-125...
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
    case WAIT_OBJECT_CLEAR:
      dungBangTai();
      if (coVat()) {
        dangDemHetVat = false;
      } else if (!dangDemHetVat) {
        dangDemHetVat = true;
        clearStartedAt = now;
      } else if (now - clearStartedAt >= CLEAR_STABLE_TIME) {
        resetNhanDien();
        lastReadAt = now;
        state = WAIT_COLOR;
        Serial.println("San sang: dat 1 vat tai vi tri doc mau. Motor dang dung.");
      }
      break;

    case WAIT_COLOR: {
      // Vật ở vị trí gắp khi chưa chốt màu: không chạy và không gắp.
      if (coVat()) {
        batDauChoVat();
        Serial.println("Vi tri gap dang co vat chua biet mau. Hay lay vat ra.");
        break;
      }
      if (now - lastReadAt < READ_INTERVAL) break;
      lastReadAt = now;

      uint16_t r, g, b, c;
      sensor.getRawData(&r, &g, &b, &c);
      // Đọc màu có thể mất thời gian; kiểm tra lại cảm biến trước khi chạy.
      if (coVat()) {
        batDauChoVat();
        break;
      }
      uint32_t tong = (uint32_t)r + g + b;
      if (tong > 0) {
        Serial.printf("R: %.1f%% | G: %.1f%% | B: %.1f%% | Clear: %u\n",
          100.0f * r / tong, 100.0f * g / tong,
          100.0f * b / tong, (unsigned)c);
      }
      ObjectColor color = (c < CLEAR_MIN || c >= RAW_MAX ||
                              r >= RAW_MAX || g >= RAW_MAX || b >= RAW_MAX)
        ? COLOR_UNKNOWN : nhanDienMau(r, g, b);
      if (color == COLOR_UNKNOWN) {
        mauTruoc = COLOR_UNKNOWN;
        soLanCungMau = 0;
      } else if (color == mauTruoc) {
        if (soLanCungMau < SO_LAN_XAC_NHAN) soLanCungMau++;
      } else {
        mauTruoc = color;
        soLanCungMau = 1;
      }
      if (soLanCungMau >= SO_LAN_XAC_NHAN) {
        mauDaChot = color;
        Serial.print("Da chot mau: ");
        Serial.println(color == COLOR_RED ? "DO" :
                       color == COLOR_GREEN ? "XANH LA" : "XANH DUONG");
        transportStartedAt = millis();
        state = TRANSPORT;
        chayBangTai();
        Serial.println("Chay bang tai, cho vat toi cam bien gap.");
      }
      break;
    }

    case TRANSPORT:
      // Không đọc lại màu, không quay SG1 khi đang vận chuyển.
      if (coVat()) {
        dungBangTai();  // Dừng ngay khi phát hiện, bỏ delay(50) của bản cũ.
        stoppedAt = now;
        state = WAIT_SETTLE;
        Serial.println("Vat toi vi tri gap. Dung bang tai, cho on dinh.");
      } else if (now - transportStartedAt >= TRANSPORT_TIMEOUT) {
        baoLoiVanChuyen();
      }
      break;

    case WAIT_SETTLE:
      dungBangTai();
      if (!coVat()) {
        // Tín hiệu thoáng qua: chưa gắp, tiếp tục đưa vật tới cảm biến.
        // Không reset thời gian vận chuyển để tránh chạy vô hạn.
        if (now - transportStartedAt >= TRANSPORT_TIMEOUT) {
          baoLoiVanChuyen();
        } else {
          state = TRANSPORT;
          chayBangTai();
        }
      } else if (now - stoppedAt >= SETTLE_TIME) {
        if (mauDaChot == COLOR_UNKNOWN) {
          batDauChoVat();
          break;
        }
        xuLyVat(mauDaChot);
        // Hoàn thành: motor vẫn dừng. Không tự chạy như bản cũ.
        batDauChoVat();
        Serial.println("Xong chu trinh. Cho vat tiep theo tai cam bien mau.");
      }
      break;

    case FAULT:
      dungBangTai();
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

  for (int i = 0; i < 4; i++) {
    danhSachServo[i]->write(gocHienTai[i]);
  }

  // Cấu hình giống bản test TCS34725 đã chạy đúng.
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);

  if (!sensor.begin(TCS34725_ADDRESS, &Wire)) {
    dungBangTai();

    Serial.println("KHONG TIM THAY TCS34725 tai 0x29!");
    Serial.println("Kiem tra nguon, SDA 21, SCL 22 va reset ESP32.");

    while (true) {
      delay(1000);
    }
  }

  Serial.println("TCS34725 ket noi thanh cong.");

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

  batDauChoVat();
  Serial.println("Khoi dong: MOTOR DUNG. Dat tung vat tai cam bien mau.");
}

// ================= VÒNG LẶP =================

void loop() {
  docLenhSerial();
  capNhatServo();

  if (cheDoTuDong) {
    capNhatTuDong();
  }
}