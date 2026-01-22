# ROS Hooks Migration Guide

## Migration to Profile-Aware ROS System

This guide explains how to migrate existing ROS hooks to use the new profile-aware system.

### Old Way (Direct topic/service usage)
```typescript
import { ROSTopicFactory } from '@/utils/ros/topics-and-services';

const topicFactory = new ROSTopicFactory(connection.ros);
const topic = topicFactory.createAndSubscribeTopic('velocity', callback);
```

### New Way (Profile-aware)
```typescript
import { useProfileAwareROS } from '@/hooks/useProfileAwareROS';

const { topicFactory, robotProfile } = useProfileAwareROS();
if (topicFactory) {
  const topic = topicFactory.createAndSubscribeTopic('velocity', callback);
}
```

## Benefits
1. **Automatic Topic/Service Mapping**: Topics and services are automatically mapped based on the robot type
2. **Profile Information**: Access to robot capabilities, dimensions, and settings
3. **Backward Compatibility**: Falls back to legacy mappings if no profile is found

## Example Hook Update

### Before:
```typescript
export function useRosCmdVel() {
  const { connection } = useRobotConnection();
  
  useEffect(() => {
    if (!connection.ros) return;
    
    const factory = new ROSTopicFactory(connection.ros);
    const topic = factory.createAndSubscribeTopic('velocity', handleVelocity);
    
    return () => topic.unsubscribe();
  }, [connection.ros]);
}
```

### After:
```typescript
export function useRosCmdVel() {
  const { topicFactory, robotProfile } = useProfileAwareROS();
  
  useEffect(() => {
    if (!topicFactory) return;
    
    const topic = topicFactory.createAndSubscribeTopic('velocity', handleVelocity);
    
    // Can also use robot profile info
    const maxVelocity = robotProfile?.capabilities.maxLinearVelocity || 1.0;
    
    return () => topic.unsubscribe();
  }, [topicFactory, robotProfile]);
}
```

## Adding New Robot Profiles

1. Create a new profile in `/src/config/robot-profiles/profiles/`
2. Add it to the registry in `/src/config/robot-profiles/index.ts`
3. The system will automatically use the new mappings when that robot type is connected