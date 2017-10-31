# machine_visualizer
経路とか機体とかの可視化パッケージ  
基本的にはraspi_developmentのノードを一緒に走らせて使う

## ノード
* steer_control：visualization_msgs::Pose2D(自己位置座標)とvisualization_msgs::Vector3(速度ベクトル)のトピックを受け取り、RViz上になんちゃって4輪独立ステアリングとして表示する。  
* path_RViz_bridge：raspi_development/make_path_CSVで生成した経路データをRViz上に表示する
* tire_test：タイヤっぽい円筒形のマーカーをRViz上に表示するだけ
