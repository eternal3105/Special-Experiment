# 災害救助全景導航系統 moveye
## Disaster Relief Panoramic Navigation System

### 情境 : 災難救助(暫定)

### meeting 時間
* **每週四下午**
  
### 主架構決定
* **網頁**
  * **做兩個按鈕改變影片展示參數**
  * **抓取地圖資訊做淡化處理展示**
  * **實作定位座標並回傳地圖端**
* **地圖**
  * **接取網頁過來的座標資料並給定參數做導航 goal_assign => get_push.py**
  * **發送地圖資料到伺服器**
  * **scp /home/rosky01/ROSKY/catkin_ws/src/rosky_slam/map/map253.pgm  rosmaster@192.168.0.91:~/inital/img/**
* **全景**
  * **目前針對高斯金字塔的層數做差別分析**   
  * **後續也可以針對用影像的幾分之幾來做融合進行分析**
  * **取得網頁傳遞過來的兩個按鈕訊息**
  * **上述方法也可以，不過也可以直接run兩個node讓網頁決定要看哪個node**

