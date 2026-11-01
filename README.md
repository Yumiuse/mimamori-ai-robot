# mimamori-ai-robot
AI-based playground safety monitoring robot simulation using ROS 2 and Turtlesim
# みまもりAI v2｜園庭の飛び出しリスク判定ロボット

松尾研 Physical AI 最終課題として制作した、ROS 2とTurtlesimによるシミュレーションです。

## 最終提出版

**[mimamori_v2.py](mimamori_v2.py)** が最終提出版です。



## 作品の概要

園児役の亀と見守りロボット役の亀を用意し、園児が門へ近づく状況を再現します。

園児役の座標から門までの距離と接近速度を計算し、k近傍法（k=3）で危険を判定します。

危険判定後、見守りロボットが園児役へ向かい、近づいたところで両方が停止します。

## 実行結果

- 約46秒：AIが危険を判定
- 約50秒：ロボットが園児役に追いつき、両方が停止

※実際の園児の画像認識や、職員への通知は未実装です。AIの見本データ19件は検証用に作成したものです。
