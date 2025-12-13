- thiết lập 2 chân ce và csn nrf24l01 bằng lệnh 
      RF24 radio(<ce>,<csn>)
- thiết lập địa chỉ cho 2 phía thu và phát
      const byte address[6] = {<mảng địa chỉ>} hoặc "<chuỗi>"
(nrf24l01 có 6 data pipe address cho phép 1rx có tối đa 6tx tương ứng với pipe 1 2 3 4 5 0)
- lệnh kiểm tra nrf có kết nối được không
      if(!radio.begin()){
          serial.print("Lỗi!");
          while(1); 
      }
- khởi tạo phía phát và thu
    phía phát: radio.openWritingPipe(addr);
    phía thu: radio.openReadingPipe(<pipe>, addr);
- cài công suất thu phát
      radio.setPALevel(<level>);
    có 4 mức công suất: rf24_pa_min(-18dBm), rf24_pa_low(-12dBm), rf24_pa_high(-6dBm) và rf24_pa_max(0dBm)
    nếu không khai báo thì mặc định công suất max(theo thư viện???)
- cài kênh truyền:
      radio.setChannel(<kênh>);
    kênh truyền tính bằng công thức 2.4GHz + <kênh>(MHz), nếu không khai báo thì mặc định là 76(theo thư viện)
- cài tốc độ truyền:
      radio.setDataRate(<tốc độ truyền>);
    có 3 mức tốc độ truyền: rf24_1mbps, rf24_2mbps, rf24_250kbps
    công suất càng cao truyền càng xa, tốc độ càng cao truyền càng ngắn
- cài đặt rx và tx:
    tx: radio.stopListening();
    rx: radio.startListening();
- hàm kiểm tra gửi nhận tín hiệu:
      radio.available() trả về 1 nếu rx nhận, 0 nếu không thấy gì
- hàm gửi tin:
      <kiểu dữ liệu> <biến> = <giá trị>
      radio.write(&<biến>, sizeof(<biến>));
    có thể gửi chuỗi, số, struct, giá trị cờ, command...
- hàm nhận tin:
      <kiểu dũ liệu> <biến>
      radio.read(&<biến>, sizeof(<biến>));
    dữ liệu nhận tương tự như gửi
























