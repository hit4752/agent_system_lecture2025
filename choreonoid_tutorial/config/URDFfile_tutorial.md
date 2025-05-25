**urdf\_tutorials.md**:

# URDFチュートリアル (ROS Wiki urdf/Tutorials)

**目次:**

1. [前提条件](#前提条件)
2. [URDFをステップ・バイ・ステップで学ぶ](#urdfをステップ・バイ・ステップで学ぶ)
3. [URDFファイル全体の解説](#urdfファイル全体の解説)
4. [URDFの学習 (C++ APIを含む)](#urdfの学習-c-apiを含む)

## 前提条件

* [tf チュートリアル](http://wiki.ros.org/tf/Tutorials)

## URDFをステップ・バイ・ステップで学ぶ

1. [URDFでロボットのビジュアルモデルをゼロから構築する](#urdfでロボットのビジュアルモデルをゼロから構築する)
   *Rvizで表示できるロボットのビジュアルモデルの構築方法を学びます。*

2. [URDFで可動ロボットモデルを構築する](#urdfで可動ロボットモデルを構築する)
   *URDFで可動ジョイントを定義する方法を学びます。*

3. [URDFモデルに物理特性と衝突プロパティを追加する](#urdfモデルに物理特性と衝突プロパティを追加する)
   *リンクに衝突と慣性の特性を追加し、ジョイントに動的特性を追加する方法を学びます。*

4. [URDFファイルをXacroで整理する](#urdfファイルをxacroで整理する)
   *Xacroを使用してURDFファイルのコード量を減らすためのテクニックを学びます。*

5. [URDFをGazeboで使用する](#urdfをgazeboで使用する)
   *Gazeboでロボットをスポーンおよび制御する方法に関する予備的チュートリアルです。*

## URDFファイル全体の解説

1. [PR2ロボットの記述を理解する](#pr2ロボットの記述を理解する)
   *このチュートリアルでは、PR2のような複雑なロボットのトップレベルURDF Xacroファイルの構成を説明します。*

## URDFの学習 (C++ APIを含む)

1. [独自のURDFファイルを作成する](#独自のurdfファイルを作成する)
   *このチュートリアルでは、自分自身のURDFロボット記述ファイルの作成を開始します。*

2. [URDFファイルをパースする](#urdfファイルをパースする)
   *このチュートリアルではURDFパーサーの使用方法を教えます。*

3. [自分のロボットでrobot\_state\_publisherを使用する](#自分のロボットでrobot_state_publisherを使用する)
   *このチュートリアルでは、robot\_state\_publisherを使ってロボットの状態をtfに公開する方法を説明します。*

4. [KDLパーサーを使い始める](#kdlパーサーを使い始める)
   *このチュートリアルでは、URDFファイルからKDLツリーを作成する方法を教えます。*

5. [robot\_state\_publisherでURDFを使用する](#robot_state_publisherでurdfを使用する)
   *このチュートリアルでは、robot\_state\_publisherを使用したURDFによるロボットモデルの完全な例を示します。まず、必要な部品をすべて含むURDFモデルを作成します。次に、JointStateと変換を配信するノードを書きます。最後に、これらすべてを組み合わせて実行します。*

---

**building\_visual.md**:

# URDFでロボットのビジュアルモデルをゼロから構築する

**説明:** Rvizで表示できるロボットのビジュアルモデルの構築方法を学びます
**キーワード:** URDF
**チュートリアルレベル:** 初級
**次のチュートリアル:** [モデルを動くようにする](#urdfで可動ロボットモデルを構築する)

このチュートリアルでは、R2D2に漠然と似たロボットのビジュアルモデルをゼロから構築します。後のチュートリアルでは、モデルに[可動関節を設定](#urdfで可動ロボットモデルを構築する)したり、[物理特性を追加](#urdfモデルに物理特性と衝突プロパティを追加する)したり、[Xacroでコードを整理](#urdfファイルをxacroで整理する)したり、[Gazeboで動かす](#urdfをgazeboで使用する)ことを学びます。しかし今は、ビジュアルな形状を正しく構築することに集中しましょう。

続行する前に、[joint\_state\_publisher](http://wiki.ros.org/joint_state_publisher)パッケージがインストールされていることを確認してください。`apt-get`で`urdf_tutorial`をインストールしていれば、このパッケージも既に含まれているはずです。もし入っていなければ、`rosdep`などでインストールを更新してください。

このチュートリアルで扱うロボットモデル（およびソースファイル）はすべて`urdf_tutorial`パッケージに含まれています。

## One Shape（1つの形状）

まず、非常にシンプルな形状から始めてみましょう。可能な限り簡単なURDFの例は以下のようになります（いわば「Hello World」に相当するものです）。

```xml
<?xml version="1.0"?>
<robot name="myfirst">
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
    </visual>
  </link>
</robot>
```

このXMLを日本語に説明すると、`myfirst`という名前のロボットで、1つのリンク（部品）だけを持ち、その視覚要素は長さ0.6メートル、半径0.2メートルのシリンダのみ、ということになります。シンプルな「Hello World」的な例にしては囲むタグが多く見えるかもしれませんが、これからもっと複雑になっていくのでご安心ください。

モデルを確認するには、`display.launch`ファイルを起動します。以下のコマンドを実行してください。

```bash
$ roslaunch urdf_tutorial display.launch model:=urdf/01-myfirst.urdf
```

このコマンドは以下の3つのことを行います。

* 指定したモデルをパラメータサーバにロードする
* `sensor_msgs/JointState`メッセージと変換（TF）を発行するノードを起動する（詳細は後述）
* RVizを設定ファイル付きで起動する

上記の`roslaunch`行は、現在のディレクトリが`urdf_tutorial`パッケージのディレクトリ（つまり`urdf`ディレクトリが現在の作業ディレクトリに直接含まれる）であることを前提としています。もしそうでない場合、`01-myfirst.urdf`への相対パスが無効となり、`roslaunch`がURDFをパラメータサーバにロードしようとした時点でエラーが発生します。

作業ディレクトリに関係なく動作させるには、少し修正した引数を使います。

```bash
$ roslaunch urdf_tutorial display.launch model:='$(find urdf_tutorial)/urdf/01-myfirst.urdf'
```

（引数の値をシングルクォートで囲んでいる点に注意してください。）

`urdf_tutorial`パッケージ以外の場所から実行する場合、チュートリアル内の例の`roslaunch`行はすべて上記のように修正する必要があります。

`display.launch`を起動すると、RVizに以下のような表示がされるはずです。



**注目すべき点:**

* Fixed Frame（固定フレーム）はグリッドの中心が位置する変換フレームです。ここでは1つしかリンクがないため、そのリンク`base_link`がFixed Frameになっています。
* Visual要素（赤いシリンダ）はデフォルトで形状の中心が原点に配置されます。そのため、シリンダの半分がグリッド下に埋まっています。

## Multiple Shapes（複数の形状）

次に、複数の形状（リンク）を追加する方法を見てみましょう。そのままリンク要素を増やすだけでは、パーサーはそれらをどこに配置すればよいか分かりません。ですので、ジョイントを追加する必要があります。ジョイント要素は柔軟な関節にも固定の関節にも使えます。まずは固定（剛体で一体化した）関節から始めましょう。

```xml
<?xml version="1.0"?>
<robot name="multipleshapes">
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
    </visual>
  </link>

  <link name="right_leg">
    <visual>
      <geometry>
        <box size="0.6 0.1 0.2"/>
      </geometry>
    </visual>
  </link>

  <joint name="base_to_right_leg" type="fixed">
    <parent link="base_link"/>
    <child link="right_leg"/>
  </joint>

</robot>
```

* 上記のとおり、0.6m x 0.1m x 0.2mの直方体（box）を追加定義しています。
* ジョイントは親要素と子要素の組み合わせで定義されています。URDFは最終的に1つのルートリンクを持つツリー構造になるため、このジョイントにより脚（right\_leg）の位置はベースリンク（base\_link）の位置に従属することになります。

試しにこのモデルを表示してみましょう。

```bash
roslaunch urdf_tutorial display.launch model:=urdf/02-multipleshapes.urdf
```



2つの形状は同じ原点を共有しているため、完全に重なって表示されています。重ならないようにするには、それぞれに**オリジン**（原点位置）を定義する必要があります。

## Origins（オリジンの設定）

R2D2の脚は本体（胴体）の上半分の側面に取り付けられています。そこで、関節のオリジン（原点）を胴体の側面上部に設定します。また、脚は中央ではなく上部で接続されるため、脚側のオリジンもオフセットする必要があります。さらに、脚を鉛直にするために脚の形状を回転させます。

```xml
<?xml version="1.0"?>
<robot name="origins">
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
    </visual>
  </link>

  <link name="right_leg">
    <visual>
      <geometry>
        <box size="0.6 0.1 0.2"/>
      </geometry>
      <origin rpy="0 1.57075 0" xyz="0 0 -0.3"/>
    </visual>
  </link>

  <joint name="base_to_right_leg" type="fixed">
    <parent link="base_link"/>
    <child link="right_leg"/>
    <origin xyz="0 -0.22 0.25"/>
  </joint>

</robot>
```

* まずジョイントのオリジンを見てみます。これは親側の座標系で指定されます。上記ではY方向に-0.22m（ロボットから見て左方向、座標軸的には右方向）、Z方向に0.25m（上方向）移動した位置です。つまり、子リンクの原点は親リンクから見て上方かつ右方に配置されます（子リンク側でのビジュアルオリジン指定に関係なく）。`rpy`（ロール・ピッチ・ヨー）属性を指定していないので、子フレームの向きは親フレームと同じになります。
* 次に脚側のビジュアルオリジンを見ると、`xyz`と`rpy`の両方を指定しています。これは視覚要素の中心位置をどこに置くかを定めます。今回は脚を上部で接続したいので、原点を下に0.3m下げ（Z軸方向-0.3m）ました。また脚の長辺をZ軸に沿うようにしたいので、Y軸回りに90度（π/2）回転させています。

```bash
roslaunch urdf_tutorial display.launch model:=urdf/03-origins.urdf
```



* 上記launchファイルでは、URDFに基づいて各リンクのTFフレームが生成されます。RVizはこれを利用して各形状を適切に配置します。
* もしURDFで定義されたリンクに対応するTFフレームが存在しない場合、その形状は原点（白色）に配置されます（参考: [関連質問](http://answers.ros.org/questions/14727/)）。

## Material Girl（マテリアル設定）

「なるほど、R2D2のシリンダは赤かったですね。でも自分のロボットやR2D2は赤くないですよ！」——確かに。そのとおりです。そこでマテリアルタグを見てみましょう。

```xml
<?xml version="1.0"?>
<robot name="materials">

  <material name="blue">
    <color rgba="0 0 0.8 1"/>
  </material>

  <material name="white">
    <color rgba="1 1 1 1"/>
  </material>

  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
      <material name="blue"/>
    </visual>
  </link>

  <link name="right_leg">
    <visual>
      <geometry>
        <box size="0.6 0.1 0.2"/>
      </geometry>
      <origin rpy="0 1.57075 0" xyz="0 0 -0.3"/>
      <material name="white"/>
    </visual>
  </link>

  <joint name="base_to_right_leg" type="fixed">
    <parent link="base_link"/>
    <child link="right_leg"/>
    <origin xyz="0 -0.22 0.25"/>
  </joint>

  <link name="left_leg">
    <visual>
      <geometry>
        <box size="0.6 0.1 0.2"/>
      </geometry>
      <origin rpy="0 1.57075 0" xyz="0 0 -0.3"/>
      <material name="white"/>
    </visual>
  </link>

  <joint name="base_to_left_leg" type="fixed">
    <parent link="base_link"/>
    <child link="left_leg"/>
    <origin xyz="0 0.22 0.25"/>
  </joint>

</robot>
```

* 本体（ベースリンク）は青になりました。"blue"というマテリアルを定義し、赤・緑・青・アルファをそれぞれ0, 0, 0.8, 1に設定しています（値は0〜1の範囲）。このマテリアルを`base_link`のvisual要素で参照しました。また、白（white）のマテリアルも同様に定義し、右脚で使用しています。
* `material`タグは`visual`要素内で直接定義することもできますし、他のリンクから参照することもできます。同じマテリアルを再定義してもエラーにはなりません。
* `material`タグの代わりに`texture`タグを使って色ではなくテクスチャ画像を指定することもできます。

```bash
roslaunch urdf_tutorial display.launch model:=urdf/04-materials.urdf
```



## Finishing the Model（モデルの仕上げ）

最後に、さらにいくつかの部品を追加してモデルを完成させます。具体的には、足（左右の補助台）、車輪、頭部を追加します。最も注目すべきは球（sphere）とメッシュ（mesh）を追加する点です。また、後のチュートリアルで使用するいくつかの部品も追加しておきます。

```xml
<?xml version="1.0"?>
<robot name="visual">

  <material name="blue">
    <color rgba="0 0 0.8 1"/>
  </material>
  <material name="black">
    <color rgba="0 0 0 1"/>
  </material>
  <material name="white">
    <color rgba="1 1 1 1"/>
  </material>

  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
      <material name="blue"/>
    </visual>
  </link>

  <link name="right_leg">
    <visual>
      <geometry>
        <box size="0.6 0.1 0.2"/>
      </geometry>
      <origin rpy="0 1.57075 0" xyz="0 0 -0.3"/>
      <material name="white"/>
    </visual>
  </link>

  <joint name="base_to_right_leg" type="fixed">
    <parent link="base_link"/>
    <child link="right_leg"/>
    <origin xyz="0 -0.22 0.25"/>
  </joint>

  <link name="right_base">
    <visual>
      <geometry>
        <box size="0.4 0.1 0.1"/>
      </geometry>
      <material name="white"/>
    </visual>
  </link>

  <joint name="right_base_joint" type="fixed">
    <parent link="right_leg"/>
    <child link="right_base"/>
    <origin xyz="0 0 -0.6"/>
  </joint>

  <link name="right_front_wheel">
    <visual>
      <origin rpy="1.57075 0 0" xyz="0 0 0"/>
      <geometry>
        <cylinder length="0.1" radius="0.035"/>
      </geometry>
      <material name="black"/>
    </visual>
  </link>
  <joint name="right_front_wheel_joint" type="fixed">
    <parent link="right_base"/>
    <child link="right_front_wheel"/>
    <origin rpy="0 0 0" xyz="0.133333333333 0 -0.085"/>
  </joint>

  <link name="right_back_wheel">
    <visual>
      <origin rpy="1.57075 0 0" xyz="0 0 0"/>
      <geometry>
        <cylinder length="0.1" radius="0.035"/>
      </geometry>
      <material name="black"/>
    </visual>
  </link>
  <joint name="right_back_wheel_joint" type="fixed">
    <parent link="right_base"/>
    <child link="right_back_wheel"/>
    <origin rpy="0 0 0" xyz="-0.133333333333 0 -0.085"/>
  </joint>

  <link name="left_leg">
    <visual>
      <geometry>
        <box size="0.6 0.1 0.2"/>
      </geometry>
      <origin rpy="0 1.57075 0" xyz="0 0 -0.3"/>
      <material name="white"/>
    </visual>
  </link>

  <joint name="base_to_left_leg" type="fixed">
    <parent link="base_link"/>
    <child link="left_leg"/>
    <origin xyz="0 0.22 0.25"/>
  </joint>

  <link name="left_base">
    <visual>
      <geometry>
        <box size="0.4 0.1 0.1"/>
      </geometry>
      <material name="white"/>
    </visual>
  </link>

  <joint name="left_base_joint" type="fixed">
    <parent link="left_leg"/>
    <child link="left_base"/>
    <origin xyz="0 0 -0.6"/>
  </joint>

  <link name="left_front_wheel">
    <visual>
      <origin rpy="1.57075 0 0" xyz="0 0 0"/>
      <geometry>
        <cylinder length="0.1" radius="0.035"/>
      </geometry>
      <material name="black"/>
    </visual>
  </link>
  <joint name="left_front_wheel_joint" type="fixed">
    <parent link="left_base"/>
    <child link="left_front_wheel"/>
    <origin rpy="0 0 0" xyz="0.133333333333 0 -0.085"/>
  </joint>

  <link name="left_back_wheel">
    <visual>
      <origin rpy="1.57075 0 0" xyz="0 0 0"/>
      <geometry>
        <cylinder length="0.1" radius="0.035"/>
      </geometry>
      <material name="black"/>
    </visual>
  </link>
  <joint name="left_back_wheel_joint" type="fixed">
    <parent link="left_base"/>
    <child link="left_back_wheel"/>
    <origin rpy="0 0 0" xyz="-0.133333333333 0 -0.085"/>
  </joint>

  <joint name="gripper_extension" type="fixed">
    <parent link="base_link"/>
    <child link="gripper_pole"/>
    <origin rpy="0 0 0" xyz="0.19 0 0.2"/>
  </joint>

  <link name="gripper_pole">
    <visual>
      <geometry>
        <cylinder length="0.2" radius="0.01"/>
      </geometry>
      <origin rpy="0 1.57075 0" xyz="0.1 0 0"/>
    </visual>
  </link>

  <joint name="left_gripper_joint" type="fixed">
    <origin rpy="0 0 0" xyz="0.2 0.01 0"/>
    <parent link="gripper_pole"/>
    <child link="left_gripper"/>
  </joint>

  <link name="left_gripper">
    <visual>
      <origin rpy="0.0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://urdf_tutorial/meshes/l_finger.dae"/>
      </geometry>
    </visual>
  </link>

  <joint name="left_tip_joint" type="fixed">
    <parent link="left_gripper"/>
    <child link="left_tip"/>
  </joint>

  <link name="left_tip">
    <visual>
      <origin rpy="0.0 0 0" xyz="0.09137 0.00495 0"/>
      <geometry>
        <mesh filename="package://urdf_tutorial/meshes/l_finger_tip.dae"/>
      </geometry>
    </visual>
  </link>
  <joint name="right_gripper_joint" type="fixed">
    <origin rpy="0 0 0" xyz="0.2 -0.01 0"/>
    <parent link="gripper_pole"/>
    <child link="right_gripper"/>
  </joint>

  <link name="right_gripper">
    <visual>
      <origin rpy="-3.1415 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://urdf_tutorial/meshes/l_finger.dae"/>
      </geometry>
    </visual>
  </link>

  <joint name="right_tip_joint" type="fixed">
    <parent link="right_gripper"/>
    <child link="right_tip"/>
  </joint>

  <link name="right_tip">
    <visual>
      <origin rpy="-3.1415 0 0" xyz="0.09137 0.00495 0"/>
      <geometry>
        <mesh filename="package://urdf_tutorial/meshes/l_finger_tip.dae"/>
      </geometry>
    </visual>
  </link>

  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.2"/>
      </geometry>
      <material name="white"/>
    </visual>
  </link>
  <joint name="head_swivel" type="fixed">
    <parent link="base_link"/>
    <child link="head"/>
    <origin xyz="0 0 0.3"/>
  </joint>

  <link name="box">
    <visual>
      <geometry>
        <box size="0.08 0.08 0.08"/>
      </geometry>
      <material name="blue"/>
    </visual>
  </link>

  <joint name="tobox" type="fixed">
    <parent link="head"/>
    <child link="box"/>
    <origin xyz="0.1814 0 0.1414"/>
  </joint>
</robot>
```

```bash
roslaunch urdf_tutorial display.launch model:=urdf/05-visual.urdf
```



* 球（sphere）の追加方法はほぼ自明でしょう。上記では以下のように記述しています。

  ```xml
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.2"/>
      </geometry>
      <material name="white"/>
    </visual>
  </link>
  ```

* ここで使用しているメッシュ（`mesh`）は、PR2ロボットから拝借したものです。それぞれ別ファイルになっており、パスを指定して読み込む必要があります。パッケージ内のメッシュファイルを指定する場合は `package://パッケージ名/パス` という形式を使います。このチュートリアルのメッシュファイルは `urdf_tutorial`パッケージ内の`meshes`フォルダに入っています。

  例として、左グリッパーの定義を以下に示します。

  ```xml
  <link name="left_gripper">
    <visual>
      <origin rpy="0.0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="package://urdf_tutorial/meshes/l_finger.dae"/>
      </geometry>
    </visual>
  </link>
  ```

  * メッシュ（`mesh`）はSTLなど様々なフォーマットを使用できます。この例ではDAE形式（Collada）を使用しています。DAE形式の場合、色データを持つことができるため、URDF側で色を指定しなくてもメッシュに組み込まれた色で表示されます。しばしばメッシュに対応するテクスチャ（.tifなど）が別ファイルとして存在し、それもメッシュフォルダ内に含まれます。
  * メッシュはスケール（大きさ）を相対的に調整したり、最大外形寸法を指定して拡大縮小することもできます。
  * 別パッケージのメッシュを参照することも可能です。例えば、`package://pr2_description/meshes/gripper_v0/l_finger.dae` のように、`pr2_description`パッケージがインストールされていればPR2のグリッパーのメッシュを使うことができます。

以上で、R2D2風のURDFモデルが完成しました。次のステップに進み、[モデルを動かせるようにする](#urdfで可動ロボットモデルを構築する) チュートリアルに進んでください。

---

**building\_movable.md**:

# URDFで可動ロボットモデルを構築する

**説明:** URDFで可動ジョイントを定義する方法を学びます
**キーワード:** URDF, ジョイント
**チュートリアルレベル:** 初級
**次のチュートリアル:** [モデルに物理特性と衝突プロパティを追加する](#urdfモデルに物理特性と衝突プロパティを追加する)

*注*: このチュートリアルを始める前に、前のチュートリアル（[URDFでロボットのビジュアルモデルをゼロから構築する](#urdfでロボットのビジュアルモデルをゼロから構築する)）を完了していることが前提となります。

このチュートリアルでは、前のチュートリアルで作成したR2D2モデルに可動関節を追加します。以前のモデルではすべてのジョイントが固定（`fixed`）でした。ここでは、新たに3種類の主要なジョイントタイプ（連続回転、回転、直動）を扱ってみましょう。

前提条件として、必要なパッケージがすべてインストール済みであることを確認してください（何が必要かは前のチュートリアルを参照）。

また、このチュートリアルで使用するロボットモデルも`urdf_tutorial`パッケージ内に含まれています。

**目次:**

1. [頭部のジョイント（Head）](#頭部のジョイントhead)
2. [グリッパーのジョイント（Gripper）](#グリッパーのジョイントgripper)
3. [グリッパーアームのジョイント（Gripper Arm）](#グリッパーアームのジョイントgripper-arm)
4. [その他のジョイントタイプ](#その他のジョイントタイプ)
5. [姿勢（Pose）の指定](#姿勢poseの指定)
6. [次のステップ](#次のステップ)

[`こちらに新しいURDFファイルがあります`](https://github.com/ros/urdf_tutorial/blob/master/urdf/06-flexible.urdf)（柔軟な関節を含む完成版）。前バージョンと見比べることで変更点が確認できますが、このチュートリアルでは3つの例となる関節に焦点を当てて解説します。

モデルを可視化して操作するには、前回と同じコマンドを実行します。ただし今回は実行すると可動関節の値を制御するGUIも表示されます。モデルをいろいろ操作してみて、動き方を観察してみてください。それでは、どのように実現したかを見ていきましょう。

（柔軟なモデルのスクリーンショット）

## 頭部のジョイント (The Head)

```xml
<joint name="head_swivel" type="continuous">
  <parent link="base_link"/>
  <child link="head"/>
  <axis xyz="0 0 1"/>
  <origin xyz="0 0 0.3"/>
</joint>
```

* 本体（base\_link）と頭部（head）の接続は「continuous（連続回転）」型のジョイントです。これは負の無限大から正の無限大まで任意の角度を取れるジョイントで、無制限回転が可能です。車輪も同様に連続回転ジョイントとしてモデル化します（両方向に無限に回転できるように）。
* このジョイントでは、追加で`axis`要素を指定しています。これは回転の軸を表すxyzベクトルで、頭部がどの軸周りに回転するかを示します。ここではZ軸周りに回転させたいので`"0 0 1"`としています。

## グリッパーのジョイント (The Gripper)

```xml
<joint name="left_gripper_joint" type="revolute">
  <axis xyz="0 0 1"/>
  <limit effort="1000.0" lower="0.0" upper="0.548" velocity="0.5"/>
  <origin rpy="0 0 0" xyz="0.2 0.01 0"/>
  <parent link="gripper_pole"/>
  <child link="left_gripper"/>
</joint>
```

* 左右のグリッパー（つまむ部分）のジョイントは`revolute`（回転ヒンジ）ジョイントとしてモデル化しました。この`revolute`ジョイントは連続回転ジョイントと似ていますが、**可動範囲に制限**があります。そのため、`limit`タグを用いてジョイントの下限・上限（ラジアン）を指定しています。また、最大速度や最大努力（トルク）も指定が必要ですが、この例では適当な値を入れておきます（シミュレーションには重要ですが、ここでは深く気にしなくて構いません）。

## グリッパーアームのジョイント (The Gripper Arm)

```xml
<joint name="gripper_extension" type="prismatic">
  <parent link="base_link"/>
  <child link="gripper_pole"/>
  <limit effort="1000.0" lower="-0.38" upper="0" velocity="0.5"/>
  <origin rpy="0 0 0" xyz="0.19 0 0.2"/>
</joint>
```

* グリッパーアームは`prismatic`（直動）ジョイントです。これは軸に沿ってスライド（平行移動）する関節で、ロボットのグリッパーアームが伸縮する動きを表現します。
* prismaticジョイントの可動範囲（制限）の指定は、回転ジョイントと同様に`limit`タグを使いますが、単位がラジアンではなくメートルになります。この例では上限0m、下限-0.38mとしています。

**その他**: prismaticとrevoluteの`limit`要素には、`effort`（努力限界）と`velocity`（速度限界）も含まれることに注意してください。上記例では`effort`を1000.0、`velocity`を0.5としましたが、これらはシミュレーションや制御において使用される値で、モデルの可視化のみでは特に意味を持ちません。

## その他のジョイントタイプ

可動ジョイントには他にも種類があります。prismaticジョイントが1軸方向の平行移動しかできないのに対し、**planar**ジョイントは平面（2次元）内を動くことができます。また、**floating**ジョイントは3次元空間内で自由に動く6自由度（位置3+姿勢3）を持つジョイントです。これらのジョイントは1つの値では制御できないため、このチュートリアルでは扱っていません（RViz上でも操作スライダは生成されません）。

## 姿勢 (Pose) の指定

スライダを動かすと、モデルがRViz内で動きます。これはどのように実現されているのでしょうか？まず、GUI（joint\_state\_publisher）はURDFを解析し、固定でないジョイントすべてとその制限値を取得します。そして、スライダの値を用いて[`sensor_msgs/JointState`](http://docs.ros.org/api/sensor_msgs/html/msg/JointState.html)メッセージを発行します。それを[`robot_state_publisher`](http://wiki.ros.org/robot_state_publisher)が受け取り、各リンク間の変換（TF）を計算します。この変換ツリーを元にRVizがすべての形状を適切な位置に描画しています。

## 次のステップ

見た目上は動くロボットモデルができました。次はモデルに[物理特性を追加](#urdfモデルに物理特性と衝突プロパティを追加する)したり、[Xacroを使ってコードを整理](#urdfファイルをxacroで整理する)してみましょう。

---

**adding\_physical.md**:

# URDFモデルに物理特性と衝突プロパティを追加する

**説明:** リンクに衝突と慣性のプロパティを追加し、ジョイントに動的特性を追加する方法を学びます
**キーワード:** URDF, 慣性 (Inertial), 動力学 (Dynamics), 衝突 (Collision)
**チュートリアルレベル:** 初級
**次のチュートリアル:** [URDFファイルをXacroで整理する](#urdfファイルをxacroで整理する)（ROS 2向けにはGazeboとの連携チュートリアルもあります）

*注*: このチュートリアルを始める前に、前のチュートリアル（[URDFでロボットのビジュアルモデルをゼロから構築する](#urdfでロボットのビジュアルモデルをゼロから構築する)と[URDFで可動ロボットモデルを構築する](#urdfで可動ロボットモデルを構築する)）を完了していることが前提となります。

このチュートリアルでは、URDFモデルに基本的な物理特性と衝突要素を追加する方法を見ていきます。

**目次:**

1. [Collision（衝突要素）](#collision衝突要素)
2. [Physical Properties（物理特性）](#physical-properties物理特性)

   1. [Inertia（慣性モーメント）](#inertia慣性モーメント)
   2. [Contact Coefficients（接触係数）](#contact-coefficients接触係数)
   3. [Joint Dynamics（ジョイント動力学）](#joint-dynamicsジョイント動力学)
3. [Other Tags（その他のタグ）](#other-tagsその他のタグ)
4. [Next Steps（次のステップ）](#next-steps次のステップ)

## Collision（衝突要素）

これまでリンクには`visual`要素しか指定していませんでした。`visual`要素はその名の通りロボットの見た目を定義します。しかし、衝突判定を行ったりGazeboなどでシミュレーションを行うためには、`collision`要素を定義する必要があります。

[`こちらに衝突と物理特性を追加した新しいURDFファイルがあります`](https://raw.githubusercontent.com/ros/urdf_tutorial/master/urdf/06-flexible.urdf)。これ以降はその内容を抜粋しながら説明します。

まず、ベースリンクに`collision`要素を追加した例を示します。

```xml
<link name="base_link">
  <visual>
    <geometry>
      <cylinder length="0.6" radius="0.2"/>
    </geometry>
    <material name="blue">
      <color rgba="0 0 .8 1"/>
    </material>
  </visual>
  <collision>
    <geometry>
      <cylinder length="0.6" radius="0.2"/>
    </geometry>
  </collision>
</link>
```

* `collision`要素はリンク内で`visual`要素と同じ階層（同じレベル）に追加しています。
* `collision`要素は`geometry`タグで形状を指定する点が`visual`と全く同じ形式です。
* `collision`にも`origin`を指定して`visual`とは別の位置・姿勢を与えることができます（省略すれば`visual`と同じ位置・姿勢とみなされます）。

ほとんどの場合、衝突形状とビジュアル形状は同じで構いません。しかし、2つの場合に違いを設けることがあります。

* **高速化のため**: メッシュ形状同士の衝突判定は、単純形状の衝突判定より計算負荷が高くなります。そのため、衝突要素ではビジュアルで使った複雑なメッシュの代わりに簡単な形状を指定することがあります。例えば、複雑な見た目のリンクでも衝突用にはシンプルな箱や円柱で近似することで計算を軽減できます。
* **安全領域の設定**: 特定の部位に物が近づかないようにしたい場合、`collision`形状を実際の物理形状より大きめに設定することがあります。例えばR2D2の頭部を保護したい場合、頭部をすっぽり覆うシリンダを衝突形状として定義し、他の物体が頭部に近づきすぎないようにする、などです。

## Physical Properties（物理特性）

シミュレーションでモデルを正しく動作させるには、ロボットのいくつかの物理特性を定義する必要があります。つまり、物理エンジン（例えばGazebo）が必要とする情報です。主にリンクの慣性パラメータと、ジョイントの動力学パラメータがあります。

### Inertia（慣性モーメント）

シミュレーションされるすべてのリンク要素には`inertial`タグが必要です。簡単な例を示します。

```xml
<link name="base_link">
  <visual>
    <geometry>
      <cylinder length="0.6" radius="0.2"/>
    </geometry>
    <material name="blue">
      <color rgba="0 0 .8 1"/>
    </material>
  </visual>
  <collision>
    <geometry>
      <cylinder length="0.6" radius="0.2"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="10"/>
    <inertia ixx="0.4" ixy="0.0" ixz="0.0" iyy="0.4" iyz="0.0" izz="0.2"/>
  </inertial>
</link>
```

* `inertial`要素もリンク要素内で、`visual`や`collision`と同じレベルに記述します。

* `mass`では質量をキログラム単位で指定します。

* `inertia`では3x3の慣性モーメント行列の各要素を指定します。対称行列であるため6要素で十分です（上三角行列の成分を列挙すればよい）。上記例では`ixx=0.4, iyy=0.4, izz=0.2`、その他の積（ixy, ixz, iyz）は0としています。

  **慣性行列の並び**（参考）:

  ```
  | ixx  ixy  ixz |
  | ixy  iyy  iyz |
  | ixz  iyz  izz |
  ```

* これらの値はモデリングソフト（例: MeshLab）から取得することもできますし、基本形状であれば数学的に計算することもできます。上記例の値はシリンダ形状の慣性モーメントをWikipediaの[慣性モーメントの一覧](https://en.wikipedia.org/wiki/List_of_moments_of_inertia)から算出したものです。

* 慣性モーメントは質量と質量の分布に依存します。簡単な近似として、質量が形状全体に均等に分布していると仮定して計算する方法があります（上記例もそれに相当します）。

* 何を設定すれば良いか分からない場合、とりあえず`ixx=iyy=izz=1e-3`（それ以外0）くらいの小さな値を入れておくこともあります（例えば0.1mの立方体で質量0.6kg程度に相当）。逆に単位行列（1.0など）は通常大きすぎる（0.1m立方体で質量600kg相当）ので避けるべきです。

* 必要に応じて`origin`タグで重心位置や慣性行列の基準フレームをずらすこともできます（デフォルトではリンク座標系と同じ）。

* 注意: 慣性に0（またはほぼ0）を設定すると、実時間制御ではモデルが不安定になり、すべてのリンクがワールド原点に潰れてしまうことがあります。これはリアルタイム制御において非常に危険なので、ゼロ慣性は避けましょう。

### Contact Coefficients（接触係数）

リンク同士が接触した際の挙動もURDFで指定できます。`collision`要素の子要素として`contact_coefficients`タグを使います。指定できる属性は3つです。

* `mu` - [摩擦係数](https://simple.wikipedia.org/wiki/Friction)
* `kp` - [剛性係数](https://en.wikipedia.org/wiki/Stiffness)
* `kd` - [減衰係数](https://en.wikipedia.org/wiki/Damping)

（※URDF上ではこれらは省略すると0にデフォルト設定されます。）

### Joint Dynamics（ジョイント動力学）

ジョイントの動きに関するパラメータは`dynamics`タグで指定できます。2つの属性があります。

* `friction` - 静止摩擦（Coulomb摩擦）係数。プリズマティックジョイントでは単位はN（ニュートン）、回転ジョイントではNm（ニュートンメートル）。
* `damping` - 粘性抵抗係数。プリズマティックジョイントでは単位はN·s/m、回転ジョイントではN·m·s/rad。

指定しない場合、これらの値はデフォルトで0になります。

## Other Tags（その他のタグ）

純粋なURDF（Gazebo拡張などを除く）において、残りのジョイント関連タグは2つあります：`calibration`（校正）と`safety_controller`（セーフティコントローラ）です。これらはこのチュートリアルでは扱いませんが、詳しくは[URDF仕様書（英語）](http://wiki.ros.org/urdf/XML/joint)を参照してください。

## Next Steps（次のステップ）

これでロボットモデルは物理的にもそれらしくなりました。次はXacroを使ってURDFコード中の面倒な数値や重複を減らしてみましょう（[URDFファイルをXacroで整理する](#urdfファイルをxacroで整理する)チュートリアルへ進んでください）。
