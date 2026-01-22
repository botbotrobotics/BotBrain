export interface BatteryState {
  header: {
    seq: number;
    stamp: {
      nanosec: number;
      sec: number;
    };
    frame_id: string;
  };
  voltage: number; // Tensão em Volts
  current: number; // Corrente em Amperes
  charge: number; // Carga atual em Ampere-hora (pode ser NaN se desconhecido)
  capacity: number; // Capacidade atual em Ampere-hora (pode ser NaN se desconhecido)
  design_capacity: number; // Capacidade de projeto em Ampere-hora (pode ser NaN se desconhecido)
  percentage: number; // Porcentagem (de 0 a 1) da bateria (pode ser NaN se desconhecido)
  power_supply_status: number; // Status da fonte de alimentação (valores definidos na mensagem)
  power_supply_health: number; // Saúde da bateria (valores definidos na mensagem)
  power_supply_technology: number; // Tecnologia da bateria (valores definidos na mensagem)
  present: boolean; // Indica se a bateria está presente
  cell_voltage: number[]; // Array com a tensão de cada célula
  cell_temperature: number[]; // Array com a temperatura de cada célula
  location: string; // Localização da bateria no robô
  serial_number: string; // Número de série da bateria
}
