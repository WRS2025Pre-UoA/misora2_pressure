# misora2_pressure
## 内容
 - 受け取った画像から圧力メーターの検出
 - メーターから針を検出し、以下の３つをオペレータPCへ渡す
    - 圧力値$[\mathrm{hPa}]$
    - 証拠画像：ボックス付き画像（圧力値を書き込む）

## コード(src/)
 - detction.cpp : Yolo8を用いたメーターの自動検出を行う
 - detction_main.cpp : detection.cppを単体で起動　あらかじめ読み取る画像パスを宣言
 ~~~bash!
 ros2 run misora2_pressure pressure_detection
 ~~~
 - pressure_component.cpp : ノード間の通信を行う ライブラリ化されている
 - pressure_node.cpp : pressure_componentを単体(ノードとして)で起動
 ~~~bash!
 ros2 run misora2_pressure pressure_node
 ~~~