export const rosPackages = {
  std: 'std_msgs',
  geometry: 'geometry_msgs',
  sensor: 'sensor_msgs',
  nav: 'nav_msgs',
  stdServices: 'std_srvs',
  customService: 'custom_interfaces/srv',
  customInterfaces: 'custom_interfaces',
  customMessages: 'custom_msgs',
  unitree: 'unitree_go/msg',
  diagnostic: 'diagnostic_msgs',
};

/**
 * Objeto que contém o nome do pacote e o tipo de retorno para cada tópico ou serviço
 * cujo as chaves ou props são os nomes dos tipos de retorno todas em lowerCase.
 */
export const rosTypes = {
  string: { pkg: rosPackages.std, type: 'String' },
  bool: { pkg: rosPackages.stdServices, type: 'SetBool' },
  float32: { pkg: rosPackages.std, type: 'Float32' },
  odometry: { pkg: rosPackages.nav, type: 'Odometry' },
  laserScan: { pkg: rosPackages.sensor, type: 'LaserScan' },
  twist: { pkg: rosPackages.geometry, type: 'Twist' },
  occupancyGrid: { pkg: rosPackages.nav, type: 'OccupancyGrid' },
  jointState: { pkg: rosPackages.sensor, type: 'JointState' },
  image: { pkg: rosPackages.sensor, type: 'CompressedImage' },
  temperature: { pkg: rosPackages.std, type: 'Float32' },
  battery: { pkg: rosPackages.sensor, type: 'BatteryState' },
  llmPrompt: { pkg: rosPackages.customService, type: 'LLMPrompt' },
  sportModeState: { pkg: rosPackages.unitree, type: 'SportModeState' },
  mode: { pkg: rosPackages.customInterfaces, type: 'Mode' },
  pose: { pkg: rosPackages.customInterfaces, type: 'Pose' },
  light: { pkg: rosPackages.customMessages, type: 'LightControl' },
  robotStatus: { pkg: rosPackages.customInterfaces, type: 'RobotStatus' },
  antiCollision: {
    pkg: rosPackages.customInterfaces,
    type: 'ObstacleAvoidance',
  },
  talk: { pkg: rosPackages.customService, type: 'Talk' },
  poseStamped: { pkg: rosPackages.geometry, type: 'PoseStamped' },
  diagnosticArray: { pkg: rosPackages.diagnostic, type: 'DiagnosticArray' },
  diagnosticStats: { pkg: rosPackages.std, type: 'String' },
};

/**
 * Retorna o nome completo do tipo de retorno dos tópicos e serviços
 * do ROS de acordo com o tipeKey informado.
 * @param typeKey O tipo de retorno.
 * @returns Uma string no formato pkg/type com o tipo de retorno, ex: "stg_msgs/Float32"
 */
export function getRosMsgType(typeKey: keyof typeof rosTypes): string {
  const { pkg, type } = rosTypes[typeKey];
  return `${pkg}/${type}`;
}

export function getRosServiceType(typeKey: keyof typeof rosTypes): string {
  const { pkg, type } = rosTypes[typeKey];
  return `${pkg}/${type}`;
}
