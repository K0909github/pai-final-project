import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
import whisper
import time

class VoiceCommanderArm(Node):
    def __init__(self):
        super().__init__('voice_commander_arm')
        
        self.publisher_ = self.create_publisher(
            JointTrajectory, 
            '/crane_plus_arm_controller/joint_trajectory', 
            10)
        
        self.get_logger().info("Whisperモデルをロードしています...")
        self.model = whisper.load_model("base")
        self.get_logger().info("モデルのロードが完了しました。")

    def transcribe_and_command(self, audio_file_path):
        self.get_logger().info(f"音声ファイル {audio_file_path} を文字起こし中...")
        result = self.model.transcribe(audio_file_path, language="ja", fp16=False)
        text = result["text"]
        self.get_logger().info(f'--> 文字起こし結果: "{text}"')
        
        if "右" in text or "ネギ" in text:
            self.get_logger().info("コマンド: アームを右に動かします")
            self.move_joint([-0.5])
        elif "左" in text:
            self.get_logger().info("コマンド: アームを左に動かします")
            self.move_joint([0.5])
        else:
            self.get_logger().info("コマンドが不明です。")

    def move_joint(self, joint_positions):
        traj_msg = JointTrajectory()
        traj_msg.joint_names = ['crane_plus_joint1'] 

        point = JointTrajectoryPoint()
        point.positions = [float(p) for p in joint_positions]
        point.time_from_start.sec = 2

        traj_msg.points.append(point)
        
        self.get_logger().info("アームにコマンドを送信します...")
        self.publisher_.publish(traj_msg)


def main(args=None):
    rclpy.init(args=args)
    voice_commander = VoiceCommanderArm()
    
    audio_file = "migi.m4a" 
    
    try:
        voice_commander.transcribe_and_command(audio_file)
        time.sleep(3) 
    except FileNotFoundError:
        voice_commander.get_logger().error(f"エラー: 音声ファイル '{audio_file}' が見つかりません。")
    except Exception as e:
        voice_commander.get_logger().error(f"エラーが発生しました: {e}")
    finally:
        voice_commander.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
