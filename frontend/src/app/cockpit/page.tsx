import ChatContainer from '@/components/chat-container';
import MapsAndCams from '@/components/robot-cams';
// import PopupsContainer from '@/components/popups-container';
import RobotControls from '@/components/robot-controls';
import RobotData from '@/components/robot-data';
import MissionsPanel from '@/components/cockpit/missions-panel';
import CockpitClient from './cockpit-client';

// Force dynamic rendering to prevent static generation issues
export const dynamic = 'force-dynamic';

export default async function RobotHome() {
  return (
    <CockpitClient>
      {/* Desktop layout (xl and up - 1280px+) */}
      <div className="hidden xl:flex w-full h-[calc(100vh-70px-12px)] items-stretch justify-between relative px-1 overflow-hidden">
        {/* Left column (30%) - data + missions + chat */}
        <div className="flex flex-col gap-2 w-[30%] px-1 pt-4 h-full pb-1">
          <div className="flex-[2] overflow-auto min-h-0">
            <RobotData />
          </div>
          <div className="flex-[2] overflow-auto min-h-0">
            <MissionsPanel />
          </div>
          <div className="flex-[3] min-h-0">
            <ChatContainer />
          </div>
        </div>

        {/* Middle column (60%) - 3D viz + cams */}
        <div className="w-[60%] pt-4 flex flex-col justify-center h-full overflow-auto pb-1 px-1">
          <div className="w-full h-full overflow-auto">
            <MapsAndCams />
          </div>
        </div>

        {/* Right column (10%) - controls */}
        <div className="w-[11%] pb-1 px-1 pt-4 h-full flex flex-col pr-4">
          <div className="w-full h-full overflow-auto">
            <RobotControls />
          </div>
        </div>
      </div>

      {/* Tablet layout (sm to xl - 640px to 1280px) - 2 rows */}
      <div className="hidden sm:flex xl:hidden w-full h-[calc(100vh-70px-12px)] flex-col overflow-hidden px-1">
        {/* First row - Maps/Cams and Chat side by side */}
        <div className="flex gap-2 h-[55%] pt-4 pb-2">
          {/* Maps and Cams - takes more space */}
          <div className="w-[65%] h-full overflow-auto">
            <MapsAndCams isMobile={true} />
          </div>
          {/* Chat - slightly shorter than mobile */}
          <div className="w-[35%] h-full overflow-auto">
            <ChatContainer />
          </div>
        </div>
        
        {/* Second row - Info, Missions and Controls side by side */}
        <div className="flex gap-2 h-[45%] pb-4">
          {/* Robot Data/Information */}
          <div className="w-[40%] h-full overflow-auto">
            <RobotData />
          </div>
          {/* Missions */}
          <div className="w-[30%] h-full overflow-auto">
            <MissionsPanel />
          </div>
          {/* Robot Controls/Buttons */}
          <div className="w-[30%] h-full overflow-auto">
            <RobotControls />
          </div>
        </div>
      </div>

      {/* Mobile layout (below sm - below 640px) - stacked sections */}
      <div className="flex sm:hidden w-full h-[calc(100vh-70px-12px)] flex-col overflow-y-auto overflow-x-hidden px-1">
        {/* Mobile layout with the requested order */}
        <div className="w-full pt-4 pb-1">
          <MapsAndCams isMobile={true} />
        </div>
        <div className="w-full pb-1">
          <RobotData />
        </div>
        <div className="w-full pb-1">
          <MissionsPanel />
        </div>
        <div className="w-full pb-1">
          <RobotControls />
        </div>
        <div className="w-full pb-4">
          <ChatContainer />
        </div>
      </div>
    </CockpitClient>
  );
}
