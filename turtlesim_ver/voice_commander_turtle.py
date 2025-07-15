import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import whisper
import time

class VoiceCommanderTurtle(Node):
    def __init__(self):
        super().__init__('voice_commander_turtle')
        
        # 亀を動かすためのPublisherを作成
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        
        self.get_logger().info("Whisperモデルをロードしています...")
        self.model = whisper.load_model("base")
        self.get_logger().info("モデルのロードが完了しました。")

    def transcribe_and_command(self, audio_file_path):
        self.get_logger().info(f"音声ファイル {audio_file_path} を文字起こし中...")
        result = self.model.transcribe(audio_file_path, language="ja", fp16=False)
        text = result["text"]
        self.get_logger().info(f'--> 文字起こし結果: "{text}"')
        
        move_cmd = Twist()
        
        if "前" in text:
            self.get_logger().info("コマンド: 前に進みます")
            move_cmd.linear.x = 2.0
        elif "後ろ" in text:
            self.get_logger().info("コマンド: 後ろに進みます")
            move_cmd.linear.x = -2.0
        elif "右" in text or "ネギ" in text:
            self.get_logger().info("コマンド: 右に曲がります")
            move_cmd.angular.z = -1.8
        elif "左" in text:
            self.get_logger().info("コマンド: 左に曲がります")
            move_cmd.angular.z = 1.8
        else:
            self.get_logger().info("コマンドが不明です。")

        self.publisher_.publish(move_cmd)
        self.get_logger().info("コマンドを送信しました。")


def main(args=None):
    rclpy.init(args=args)
    voice_commander = VoiceCommanderTurtle()
    
    audio_file = "migi.m4a" 
    
    try:
        voice_commander.transcribe_and_command(audio_file)
    except Exception as e:
        voice_commander.get_logger().error(f"エラーが発生しました: {e}")
    finally:
        voice_commander.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
