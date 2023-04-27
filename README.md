# ros_central_servers2仕様書
## 著者：大島　航
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

`rcs2` は、リモート制御システムのためのプロジェクトです。このパッケージは、独自のプロトコルを使用してrobotを簡単に操作できるように設計されています。

## 特徴

- シンプルなAPIでリモートデバイスの制御が可能
- デバイス間でのデータ送信を容易に行える
- 独自のプロトコルを使用し、高速で安全な通信を実現
- クライアント/サーバーアーキテクチャで複数デバイスを一元管理可能


## 使い方(server side)

基本的な使い方は以下の通りです。

1. ros_central_servers2をクローンします。

```
git clone https://github.com/TechShare-inc/ros_central_servers2.git
```

2. サーバーのディレクトリに移動します

```
cd ros_central_servers/RCS_Server
```

3. dockerを用いて、RCSサーバーを立ち上げます。

```
./build_and_run.sh <the ip of thits server>
```
*サーバーのIPを指定しない場合、自動的にlocal hostのipが自動的に割り当てられます。例えば、サーバーに割り当てたいipが192.168.1.25だった場合、以下の用になります。
```
./build_and_run.sh 192.168.1.25
```


4. ブラウザから、サーバーのUIにアクセスします。

ブラウザを開き、アドレスバーに以下のようにアドレスとポート番号を入力します。（IPが192.168.1.25の場合）
```
http://192.168.1.25:5000
```

## 使い方(client side)
1. ros_central_servers2をクローンします。

```
git clone https://github.com/TechShare-inc/ros_central_servers2.git
```
2. buildします。（ワークスペース内に、techshare_ros_pkg2がある前提）
```
cd <your workspace>
coclon build –symlink-install —packages-select rcs_client
```
3. nodeを立ち上げます。（RCS_clientは環境変数で必須の項目があるので、bashrcに記述するか、以下のようにノードを立ち上げる前に、指定してください）
```
export RCS_SERVER_ADDRESS=localhost
export ROBOT_NAME=TB3_01 
export MAP_ID=sh
ros2 launch rcs_client tb3.launch.py
```
RCS_SERVER_ADDRESSは上述したサーバーのアドレスを指定してください
ROBOT_NAMEは任意の名前
MAP_IDはそのロボットが所属するMapの名前を指定してください。

*Chome, firefox, MicrosoftEdgeで動作確認しています。

## catmuxによる一括起動
### catmuxとは
Catmuxは、ターミナルマルチプレクサーの1つで、複数のターミナルを単一のターミナルに統合して管理することができます。Catmuxはtmuxの上位互換で、tmuxの一部の機能に加えて、独自の機能を提供します。

Catmuxは、LinuxとmacOSの両方で動作します。Catmuxは、リモートサーバーへのSSH接続や、複数のプロセスを管理する場合など、複数のターミナルを効率的に管理する必要がある場合に役立ちます。

Catmuxの主な機能は以下の通りです。

- 複数のターミナルを単一のターミナルで管理
- ウィンドウとペインを柔軟に管理
- ペインのレイアウトを自由にカスタマイズ可能
- コマンドを分割して実行することができる
- マウスをサポート
- カスタマイズ可能なキーバインディング

Catmuxは、リモートサーバーのターミナルを管理するために非常に便利です。また、複数のプロセスを管理する場合にも役立ちます。Catmuxは、簡単にインストールすることができ、ユーザーフレンドリーなインターフェイスを提供します。


例：）複数のロボットを同時に表示させる場合

1. catmuxディレクトリに移動
```
cd ros_central_servers/catmux
```
2. セッションを作成する。
```
catmux_create_session ros2_two_robots_simulation.yaml
```
*session作成後、GAZEBOと、ナビゲーション、サーバー、クライアントノードがそれぞれ立ち上がります。


#### 以上、詳しい使い方やAPIについては、大島まで！

## 貢献

プロジェクトに貢献する場合は、以下の手順を行ってください。

1. リポジトリをフォークする
2. 新しいブランチを作成する (`git checkout -b my-new-feature`)
3. 変更をコミットする (`git commit -am 'Add some feature'`)
4. ブランチにプッシュする (`git push origin my-new-feature`)
5. プルリクエストを作成する

## ライセンス

このプロジェクトは MIT ライセンスの下で公開されています。詳細については、[LICENSE](LICENSE) ファイルを参照してください。


