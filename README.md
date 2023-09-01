# 災害救助全景導航系統 moveye
## Disaster Relief Panoramic Navigation System

### 情境 : 輻射(暫定)
* 功能1 : 搜索及建構地圖
  * 依靠全景圖及靈活的搖桿控車系統，可以快速地在災害區域找到受難者
  * 在搜救的同時可以建構地圖，提高對周圍環境的把握
* 功能2 : 標註
  * 在搜索過程中，可以隨時標記系統當前位置，以利後續物資傳遞
* 功能3 : 導航
  * 在確認目標位置後，可以讓自走車自主走向 **標註點** ，方便運送物資


### meeting 時間
* **每週四下午**
  
### 主架構決定
* **網頁**
  * **實現全景畫面顯示**
  * **抓取地圖資訊做淡化處理展示**
  * **實作定位座標並回傳地圖端**
* **地圖**
  * **接取網頁過來的座標資料並給定參數做導航**
  * **發送地圖資料到伺服器**
  * **scp /home/rosky01/ROSKY/catkin_ws/src/rosky_slam/map/map253.pgm  rosmaster@192.168.0.91:~/inital/img/**
* **全景**
  * **目前針對影像比例做差別分析**   
  * **上述方法也可以，不過也可以直接run兩個node讓網頁決定要看哪個node**

