AI Voice-Controlled ROS 2 Robot using Whisper
This project demonstrates how to control a ROS 2 robot using voice commands, powered by OpenAI's robust speech recognition model, "Whisper."

概要 (Overview)
このリポジトリは、AI音声認識モデル「Whisper」を活用し、ROS 2で動作するロボットを声で直感的に操作するシステムの実装です。プロジェクトの核心は、AIによる音声認識とロボット制御の連携にあり、その実現性を検証するために2つの異なるバージョンを格納しています。

turtlesim版（完成版）: 軽量なturtlesimシミュレータを使用し、システムの核となる機能を安定して実演できる完成版です。

Gazebo + ロボットアーム版（挑戦版）: より現実に近い物理シミュレータGazeboとロボットアームCRANE+ V2を用いた、応用的なバージョンです。開発過程で直面した課題とそのアプローチを示すものとして、コードを同梱しています。

✨ 特徴 (Features)
高精度な音声認識: OpenAIのWhisperモデルにより、日本語の音声コマンドを正確にテキスト化します。

ROS 2連携: 認識したテキストをROS 2のメッセージに変換し、パブリッシャーを介してロボットを制御します。

2種類のシミュレーション環境: シンプルで安定したturtlesimと、より高度なGazeboの2つの環境に対応しています。

🔧 必要なもの (Requirements)
ROS 2 (Humble Hawksbill推奨)

Python 3.8+

OpenAI Whisper (pip install openai-whisper)

turtlesim パッケージ (sudo apt-get install ros-humble-turtlesim)

(オプション) CRANE+ V2 Gazeboシミュレーション環境

マイクまたは音声ファイル (m4a形式)

🚀 実行方法 (Usage)
1. turtlesim版 (推奨)
このバージョンは軽量で安定しており、プロジェクトの核心機能を確実に試すことができます。

1-1. turtlesimの起動
新しいターミナルを開き、以下のコマンドでturtlesimを起動します。

Bash

ros2 run turtlesim turtlesim_node
1-2. 音声ファイルの準備
「前」「右」「左」「後ろ」などと録音した音声ファイル（m4a形式）をturtlesim_verフォルダ内に配置してください。

1-3. スクリプトの実行
turtlesim_ver/voice_commander_turtle.py内のaudio_file変数を、用意した音声ファイル名に更新します。その後、以下のコマンドでスクリプトを実行してください。

Bash

cd turtlesim_ver
python3 voice_commander_turtle.py
実行後、音声コマンドに応じた亀の動きがturtlesimウィンドウで確認できます。

2. Gazebo + ロボットアーム版 (挑戦版)
注意: このバージョンは物理シミュレーションが不安定なため、お使いの環境によっては正常に動作しない可能性があります。

2-1. Gazeboシミュレータの起動
CRANE+ V2のGazebo環境を起動します。

Bash

ros2 launch crane_plus_gazebo crane_plus_with_table.launch.py
2-2. 音声ファイルの準備
gazebo_verフォルダ内に、制御用の音声ファイルを配置します。

2-3. スクリプトの実行
gazebo_ver/voice_commander_arm.py内のファイルパスを適宜修正し、以下のコマンドで実行します。

Bash

cd gazebo_ver
python3 voice_commander_arm.py
🛠️ 開発過程と考察 (Development & Considerations)
当初の計画では、より現実に近いGazebo環境でのロボットアーム制御を目指して開発を開始しました。しかし、開発を進める中で、ロボットアームの挙動が物理シミュレーションの不安定性の影響を大きく受けるという課題に直面しました。

この問題の原因を切り分ける過程で、本課題の最も重要な目的は**「AIの音声認識結果を用いて、意図通りにロボットを制御するロジックを確立すること」**であると再定義しました。

そこで、物理的な不安定要素を排除し、AIとROS 2の連携という核心部分を明確に示すため、よりシンプルで安定したturtlesim環境で最終的な実装を完成させるという戦略的判断を下しました。このアプローチにより、AIの推論結果を確実かつ安定的にロボットの動作に繋げるシステムの構築に成功し、プロジェクトの目的を達成することができました。Gazebo版のコードは、その試行錯誤の過程を示す貴重な資料としてリポジトリに残しています。
