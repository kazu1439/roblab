# roblab
ロボ研のライブラリと汎用パッケージ

使用手順

1．srcの下にroblabをクローン（コピー）

2．roblabを使用するパッケージのCMakeLists.txt内、find_packageにroblabを追加する。

3．roblabを使用するパッケージのpackage.xml内に<build_depend>roblab</build_depend>を追加する。

4．buildする。

5．source devel/setup.bash

ライブラリを使用する場合
include <roblab/ヘッダー名>とする。
具体例：include <roblab/pid.h>

カスタムmsgを使用する場合
include <roblab/メッセージタイプ>とする。
具体例：include <roblab/JoyJoy.h>

joyjoyプログラムの使用方法
roblab/launch/の下にあるファイルを適宜書き換えてlaunchする。
