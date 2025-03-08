import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import time
import numpy as np


class PerformanceMetricsAnalyzer(Node):
    def __init__(self):
        super().__init__('performance_metrics_analyzer')
        self.subscription = self.create_subscription(Float32, 'motor_output', self.listener_callback, 10)
        self.publisher = self.create_publisher(Float32, 'motor_input_1', 10)
        self.timer = self.create_timer(0.2, self.publish_step_signal)  # Guardar referencia del timer
        self.start_time = time.time()
        self.step_value = 1.0
        self.time_data = []
        self.output_data = []
        self.msg = Float32()
        self.experiment_completed = False

    def publish_step_signal(self):
        current_time = time.time() - self.start_time
        
        if current_time < 20.0 and not self.experiment_completed:
            self.msg.data = self.step_value
            self.publisher.publish(self.msg)
            self.get_logger().info(f'Publicando motor_input: {self.msg.data}')
        elif not self.experiment_completed:
            # Finalizar experimento después de 20 segundos
            self.experiment_completed = True
            self.compute_performance_indices()
            self.timer.cancel()  # Detener el timer
            self.finish()  # Apagar motor
            self.destroy_node()  # Destruir nodo
            rclpy.shutdown()

    def finish(self):
        self.msg.data = 0.0
        self.publisher.publish(self.msg)
        self.get_logger().info('Motor detenido: 0.0')

    def listener_callback(self, msg):
        if not self.experiment_completed:
            current_time = time.time() - self.start_time
            self.time_data.append(current_time)
            self.output_data.append(msg.data)
            self.get_logger().info(f'Recibido motor_output: {msg.data}')

    def compute_performance_indices(self):
        if not self.time_data or not self.output_data:
            return

        y = np.array(self.output_data)
        t = np.array(self.time_data)
        
        try:
            y_final = np.mean(y[-10:])  # Valor final promedio
        except:
            y_final = y[-1]

        e_ss = abs(self.step_value - y_final)
        
        # Calcular tiempo de establecimiento (2% de margen)
        ts = t[-1]
        for i in reversed(range(len(t))):
            if abs(y[i] - y_final) > 0.03 * y_final:
                ts = t[i]
                break
        
        # Calcular sobrepaso máximo
        y_max = np.max(y)
        tp = t[np.argmax(y)]
        Mp = ((y_max - y_final) / y_final) * 100 if y_max > y_final else 0
        
        # Calcular métricas de error
        ise = np.trapz((self.step_value - y)**2, t)
        iae = np.trapz(abs(self.step_value - y), t)
        itse = np.trapz(t * (self.step_value - y)**2, t)
        itae = np.trapz(t * abs(self.step_value - y), t)

        # Guardar métricas
        with open("performance_metrics.txt", "w") as f:
            f.write(f'Tiempo de establecimiento: {ts:.4f}s\n')
            f.write(f'Sobrepaso máximo: {Mp:.2f}%\n')
            f.write(f'Tiempo de pico: {tp:.4f}s\n')
            f.write(f'Error en estado estacionario: {e_ss:.4f}\n')
            f.write(f'ISE: {ise:.4f}\n')
            f.write(f'IAE: {iae:.4f}\n')
            f.write(f'ITSE: {itse:.4f}\n')
            f.write(f'ITAE: {itae:.4f}\n')

        self.get_logger().info("Métricas guardadas en performance_metrics.txt")


def main(args=None):
    rclpy.init(args=args)
    performance_analyzer = PerformanceMetricsAnalyzer()
    
    try:
        rclpy.spin(performance_analyzer)
    except KeyboardInterrupt:
        pass
    finally:
        performance_analyzer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()