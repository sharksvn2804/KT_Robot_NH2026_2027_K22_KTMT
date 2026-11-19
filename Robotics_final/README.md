# Mô phỏng xe bám tuyến bằng Pure Pursuit

Chương trình mô phỏng xe bám tuyến tham chiếu. Pure Pursuit tính góc lái, bộ PID điều khiển vận tốc và mô hình bicycle cập nhật chuyển động của xe. Giao diện hiển thị quỹ đạo, vận tốc và sai số bám tuyến.

## 1. Cấu trúc chương trình

| File | Chức năng |
| --- | --- |
| `main.py` | Khởi động chương trình, mở giao diện và xuất kết quả. |
| `pure_pursuit.py` | Chọn điểm mục tiêu phía trước và tính góc lái. |
| `pid.py` | Tính lệnh gia tốc và vận tốc đặt khi vào cua. |
| `trajectory.py` | Tạo tuyến tham chiếu, kiểm tra vượt biên và vẽ quỹ đạo. |
| `parameters.py` | Chứa các tham số mô phỏng và hệ số điều khiển. |

## 2. Cài đặt và chạy

Cần **Python 3.9 trở lên**. Giao diện sử dụng **Tkinter**.

Mở terminal trong thư mục `Path_tracking`, nơi có `main.py` và bốn file Python còn lại. Cài thư viện và chạy chương trình:

```bash
python -m pip install numpy matplotlib
python main.py
```

## 3. Sử dụng giao diện

1. Mở menu **☰ Quỹ đạo** để chọn tuyến thẳng, chữ L, bậc thang hoặc zíc zắc.
2. Chỉnh các tham số như vận tốc hành trình, chiều rộng hành lang hoặc hệ số PID.
3. Bấm **Chay lai / Ap dung** để áp dụng tham số, sau đó bấm **Chay / Tiep tuc**.
4. Quan sát quỹ đạo xe, đồ thị vận tốc và sai số bám tuyến. Bấm **Xuat CSV** để lưu dữ liệu.

Mặc định, `Kp = 1.2`, `Ki = 0` và `Kd = 0`, nên bộ điều tốc hoạt động ở chế độ **P**. Để thử đủ ba thành phần PID, có thể bắt đầu với `Kp = 1.2`, `Ki = 0.05` và `Kd = 0.02`.

## 4. Chạy kiểm tra và so sánh không cần giao diện

Chạy bài kiểm tra có sẵn:

```bash
python main.py --self-test
```

So sánh hai lượt chạy bật và tắt giảm tốc theo độ cong với cùng bộ hệ số PID:

```bash
python main.py --compare --mode speed --kp 1.2 --ki 0.05 --kd 0.02
```

Kết quả được lưu trong thư mục `pp_comparison_output`, gồm đồ thị PNG, dữ liệu CSV, báo cáo `doi_chieu.txt` và tham số `parameters.json`.
