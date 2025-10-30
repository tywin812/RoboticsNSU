import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from action_cleaning_robot.action import CleaningTask


class CleaningActionClient(Node):
    def __init__(self):
        super().__init__('cleaning_action_client')
        self._action_client = ActionClient(self, CleaningTask, 'cleaning_task')
        self._goal_handle = None
        self._result_available = False
        self.get_logger().info('Cleaning Action Client started')

    def send_goal(self, task_type, area_size=0.0, target_x=0.0, target_y=0.0):
        goal_msg = CleaningTask.Goal()
        goal_msg.task_type = task_type
        goal_msg.area_size = area_size
        goal_msg.target_x = target_x
        goal_msg.target_y = target_y

        self.get_logger().info(f'Sending goal: {task_type}, area_size: {area_size}')
        
        self._action_client.wait_for_server()
        self._result_available = False
        
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            self._result_available = True
            return

        self.get_logger().info('Goal accepted')
        self._goal_handle = goal_handle
        
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: success={result.success}, '
                             f'cleaned_points={result.cleaned_points}, '
                             f'total_distance={result.total_distance:.2f}')
        self._result_available = True

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Feedback: progress={feedback.progress_percent}%, '
                             f'cleaned={feedback.current_cleaned_points}, '
                             f'pos=({feedback.current_x:.2f}, {feedback.current_y:.2f})')

    def wait_for_result(self):
        while not self._result_available:
            rclpy.spin_once(self, timeout_sec=0.1)


def print_menu():
    print("CLEANING ROBOT ACTION CLIENT")
    print("1. clean_square <size>  - Очистить квадрат (например: 1 3.0)")
    print("2. return_home <x> <y>  - Вернуться домой (например: 3 5.5 5.5)")
    print("4. help                 - Показать справку")
    print("5. exit                 - Выход")

def main(args=None):
    rclpy.init(args=args)
    
    action_client = CleaningActionClient()
    
    try:
        action_client.get_logger().info('Cleaning Action Client started')
        print_menu()
        
        while True:
            try:
                user_input = input("\nВведите команду: ").strip()
                
                if not user_input:
                    continue
                
                parts = user_input.split()
                command = parts[0].lower()
                
                if command == '1' or command == 'clean_square':
                    if len(parts) < 2:
                        print("Ошибка: укажите размер квадрата. Например: 1 3.0")
                        continue
                    try:
                        size = float(parts[1])
                        action_client.get_logger().info(f'\nЗадача: Очистить квадрат {size}x{size}')
                        action_client.send_goal('clean_square', area_size=size)
                        action_client.wait_for_result()
                    except ValueError:
                        print("Ошибка: размер должен быть числом")
                
                elif command == '2' or command == 'return_home':
                    if len(parts) < 3:
                        print("Ошибка: укажите координаты. Например: 3 5.5 5.5")
                        continue
                    try:
                        target_x = float(parts[1])
                        target_y = float(parts[2])
                        action_client.get_logger().info(f'\nЗадача: Вернуться домой ({target_x}, {target_y})')
                        action_client.send_goal('return_home', target_x=target_x, target_y=target_y)
                        action_client.wait_for_result()
                    except ValueError:
                        print("Ошибка: координаты должны быть числами")
                
                elif command == '4' or command == 'help':
                    print_menu()
                
                elif command == '5' or command == 'exit' or command == 'quit':
                    action_client.get_logger().info('Завершение работы...')
                    break
                
                else:
                    print(f"Неизвестная команда: {command}")
                    print("Введите 'help' для справки или 'exit' для выхода")
            
            except KeyboardInterrupt:
                print("\n\nПрограмма прервана пользователем")
                break
        
    finally:
        action_client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()