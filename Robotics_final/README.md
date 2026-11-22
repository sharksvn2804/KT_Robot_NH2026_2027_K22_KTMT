# MÔ PHỎNG ROBOT BÁM QUỸ ĐẠO SỬ DỤNG PURE PURSUIT

## 1. Giới thiệu

Chương trình mô phỏng robot bám quỹ đạo trong môi trường hành lang, sử dụng thuật toán **Pure Pursuit** để tính góc lái, kết hợp bộ điều khiển **P/PID** để điều chỉnh vận tốc.

Chương trình thực hiện so sánh 3 cấu hình điều khiển:

- **Đối tượng 1:** Pure Pursuit + P.
- **Đối tượng 2:** Pure Pursuit + PID.
- **Đối tượng 3:** Pure Pursuit + PID kết hợp giảm tốc theo độ cong.

Các bộ điều khiển được đánh giá trên 3 kịch bản: đường thẳng, chữ L (1 cua 90°) và zíc zắc (5 cua 90°).

Chương trình cung cấp giao diện mô phỏng, biểu đồ quỹ đạo, vận tốc, sai số bám đường và kết quả đánh giá định lượng.

## 2. Cấu trúc thư mục

```text
Robotics_final/
├── README.md
└── Path_tracking/
    ├── main.py
    ├── experiment.py
    ├── parameters.py
    ├── pure_pursuit.py
    ├── pid.py
    ├── trajectory.py
    ├── utils/
    │   ├── __init__.py
    │   └── angle.py
    └── results/
        ├── straight/
        ├── elbow/
        ├── zigzag/
        └── tong_hop_9_luot_chay.csv
```

| File | Chức năng |
|---|---|
| `main.py` | Khởi động giao diện, mô phỏng và điều phối chạy chương trình |
| `experiment.py` | Chạy thí nghiệm 3 đối tượng, xuất biểu đồ, GIF và CSV |
| `parameters.py` | Khai báo các thông số robot, bộ điều khiển và quỹ đạo |
| `pure_pursuit.py` | Chọn điểm nhìn trước và tính góc lái |
| `pid.py` | Điều khiển vận tốc P/PID và điều chỉnh tốc độ theo độ cong |
| `trajectory.py` | Tạo quỹ đạo, kiểm tra hành lang và hỗ trợ vẽ kết quả |
| `utils/angle.py` | Các hàm tiện ích xử lý góc |

## 3. Cài đặt môi trường

**Yêu cầu:** Python 3.9 trở lên, hỗ trợ Windows hoặc Linux.

Mở Terminal trong thư mục `Robotics_final`, sau đó chuyển đến thư mục mã nguồn:

```bash
cd Path_tracking
```

Cài đặt các thư viện cần thiết:

```bash
python -m pip install numpy matplotlib pillow
```

Các thư viện có chức năng:

- `numpy`: Xử lý dữ liệu và tính toán số học.
- `matplotlib`: Vẽ quỹ đạo và các biểu đồ.
- `pillow`: Tạo ảnh động GIF.

Giao diện sử dụng Tkinter. Trên Windows, Tkinter thường đi kèm bản cài đặt Python thông thường.

## 4. Hướng dẫn chạy chương trình

### 4.1. Chạy giao diện mô phỏng

Trong thư mục `Path_tracking`, thực hiện:

```bash
python main.py
```

Sau khi khởi động, giao diện cho phép:

- Lựa chọn quỹ đạo mô phỏng.
- Thay đổi các tham số robot và bộ điều khiển.
- Quan sát chuyển động của robot.
- Theo dõi vận tốc và sai số bám đường.
- Xuất dữ liệu kết quả mô phỏng.

### 4.2. Chạy toàn bộ 9 lượt thí nghiệm

Để tự động chạy cả 3 đối tượng trên 3 kịch bản, sử dụng:

```bash
python main.py --experiment-all
```

Chương trình lần lượt thực hiện:

1. Kịch bản đường thẳng.
2. Kịch bản chữ L (1 cua 90°).
3. Kịch bản zíc zắc (5 cua 90°).

Mỗi kịch bản chạy 3 bộ điều khiển với cùng điều kiện mô phỏng để phục vụ việc đối chiếu.

Khi hoàn tất, Terminal hiển thị:

```text
Đang chạy kịch bản: Đường thẳng
Đang chạy kịch bản: Chữ L · 1 cua
Đang chạy kịch bản: Zíc zắc · 5 cua
Hoàn tất. Kết quả: .../Path_tracking/results
Đã lưu kết quả trong thư mục results/
```

### 4.3. Kiểm tra chương trình

Chạy các bài kiểm thử có sẵn:

```bash
python main.py --self-test
```

Lệnh này dùng để kiểm tra một số chức năng mô phỏng và so sánh mà không cần mở giao diện.

## 5. Kết quả mô phỏng

Sau khi thực hiện lệnh `python main.py --experiment-all`, chương trình lưu kết quả vào thư mục `results/`.

Mỗi kịch bản có thư mục riêng:

- `straight/`: Đường thẳng.
- `elbow/`: Chữ L.
- `zigzag/`: Zíc zắc.

Cấu trúc kết quả của mỗi kịch bản:

```text
results/
└── elbow/
    ├── pp_p.gif
    ├── pp_pid.gif
    ├── pp_pid_curve.gif
    ├── 01_quy_dao_3_doi_tuong.png
    ├── 02_sai_so_va_van_toc.png
    ├── 03_bang_chi_tieu.png
    ├── pp_p.csv
    ├── pp_pid.csv
    ├── pp_pid_curve.csv
    ├── chi_tieu_danh_gia.csv
    ├── doi_chieu_theo_tien_do_s.csv
    ├── tham_so.json
    └── bao_cao.txt
```

### 5.1. Ảnh động GIF

| File | Nội dung |
|---|---|
| `pp_p.gif` | Robot sử dụng Pure Pursuit + P |
| `pp_pid.gif` | Robot sử dụng Pure Pursuit + PID |
| `pp_pid_curve.gif` | Robot sử dụng Pure Pursuit + PID và giảm tốc theo độ cong |

Các ảnh GIF thể hiện chuyển động của từng đối tượng trên cùng quỹ đạo tham chiếu.

### 5.2. Biểu đồ kết quả PNG

| File | Nội dung |
|---|---|
| `01_quy_dao_3_doi_tuong.png` | So sánh quỹ đạo di chuyển thực tế của 3 đối tượng |
| `02_sai_so_va_van_toc.png` | So sánh sai số bám đường và vận tốc |
| `03_bang_chi_tieu.png` | Biểu diễn các chỉ tiêu đánh giá định lượng |

### 5.3. Dữ liệu CSV và báo cáo

- `pp_p.csv`, `pp_pid.csv`, `pp_pid_curve.csv`: Dữ liệu trạng thái và chuyển động của từng đối tượng.
- `chi_tieu_danh_gia.csv`: Bảng chỉ tiêu đánh giá của 3 đối tượng.
- `doi_chieu_theo_tien_do_s.csv`: Đối chiếu kết quả theo cùng tiến độ tuyến tham chiếu.
- `tham_so.json`: Lưu cấu hình tham số mô phỏng.
- `bao_cao.txt`: Báo cáo kết quả bằng văn bản.
- `tong_hop_9_luot_chay.csv`: Bảng tổng hợp toàn bộ 9 lượt thí nghiệm.

## 6. Các chỉ tiêu đánh giá

Chương trình sử dụng các chỉ tiêu sau để đánh giá và so sánh hiệu quả điều khiển:

| Chỉ tiêu | Ý nghĩa |
|---|---|
| RMS(t) | Sai số bám đường theo thời gian |
| RMS(ds) | Sai số bám đường theo quãng đường xe di chuyển |
| RMS(s) | Sai số trên cùng miền tiến độ tuyến tham chiếu |
| e_max | Sai số bám đường lớn nhất |
| Thời gian | Thời gian kết thúc mô phỏng |
| Khoảng cách đích | Khoảng cách còn lại tới điểm đích |
| Vận tốc cuối | Vận tốc robot khi kết thúc |
| Vượt biên | Kiểm tra thân xe có vượt khỏi hành lang hay không |

Việc đối chiếu theo tiến độ tuyến `s` giúp so sánh các bộ điều khiển tại những vị trí tương ứng trên đường tham chiếu, đặc biệt khi các đối tượng di chuyển với vận tốc khác nhau.

## 7. Thông số mô phỏng mặc định

| Thông số | Giá trị |
|---|---|
| Chiều dài cơ sở xe | 2,9 m |
| Chiều rộng thân xe | 2,0 m |
| Góc lái tối đa | 45° |
| Vận tốc hành trình | 1,0 m/s |
| Khoảng nhìn trước cơ sở | 2,5 m |
| Hệ số nhìn trước | 0,1 s |
| Chiều rộng hành lang | 8,0 m |
| Chiều dài mỗi đoạn đường | 12,0 m |
| Bước thời gian mô phỏng | 0,05 s |
| Thời gian mô phỏng tối đa | 240 s |

Các tham số có thể được thay đổi trong giao diện hoặc tại file `parameters.py`.

## 8. Lưu ý

- Chạy các lệnh trong thư mục `Path_tracking`, nơi chứa `main.py`.
- Kết quả được lưu trong thư mục `results/`.
- Các ảnh GIF có thể mở trực tiếp bằng trình duyệt hoặc trình xem ảnh hỗ trợ GIF.
- Thông số mô phỏng có thể ảnh hưởng đến sai số, thời gian hoàn thành và khả năng vượt biên.
- Phần giảm tốc theo độ cong là cơ chế mở rộng của Pure Pursuit, chưa bao gồm đầy đủ tất cả chức năng của Regulated Pure Pursuit trong ROS 2 Navigation2.

---

**Mục đích dự án:** Mô phỏng, phân tích và đánh giá khả năng bám quỹ đạo của robot khi sử dụng thuật toán Pure Pursuit kết hợp các phương pháp điều khiển vận tốc khác nhau.
