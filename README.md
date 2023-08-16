# turtlebot專用分支

## 目前進度
 - 能原地自轉 by : turn.py
   - 角度準確到個位數誤差(<1度)
   - 給定參數 target_angle:= x 能指定自轉角度
   - 車頭方向為180車尾方向為0 +90右轉 -90左轉
     
 - 能以一定半徑做公轉 by : circle.py 
   - 給定參數 target_angle:= x 能指定公轉角度
   - 目前只能逆時正旋轉

## 待完成工作
 - circle.py 要根據給出的數據決定要左轉還是右轉
 - 直線前進部分尚為實作
   - 如果把直線當作一個半徑無限大的圓的其中一小段圓周能否實作?
