import type { Metadata, Viewport } from 'next';
import { Work_Sans } from 'next/font/google';
import './globals.css';
import { RobotConnectionProvider } from '../contexts/RobotConnectionContext';
// import { ROS2SimulatorProvider } from '@/utils/ros/ros2-simulator-provider';
import { NotificationsProvider } from '../contexts/NotificationsContext';
import { SupabaseProvider } from '@/contexts/SupabaseProvider';
import { LanguageProvider } from '@/contexts/LanguageContext';
import { ThemeProvider } from '@/contexts/ThemeContext';
import { SpeedModeProvider } from '@/contexts/SpeedModeContext';
import { SpeedInsights } from '@vercel/speed-insights/next';
import PopupsContainer from '@/components/popups-container';
import { HeaderProvider } from '@/contexts/HeaderContext';
import { RobotCustomModeProvider } from '@/contexts/RobotCustomModesContext';
import { RobotProfileProvider } from '@/contexts/RobotProfileContext';
import { ConfirmationDialogProvider } from '@/contexts/ConfirmationDialogContext';
import { ActiveMissionProvider } from '@/contexts/ActiveMissionContext';
import { NavigationTargetsProvider } from '@/contexts/NavigationTargetsContext';
// Import ROSLIB extensions for CBOR support
import '@/interfaces/RosLibExtensions';

const workSans = Work_Sans({
  subsets: ['latin'],
  weight: ['400', '500', '600', '700'],
  variable: '--font-work-sans',
  display: 'swap',
});

export const metadata: Metadata = {
  title: 'BotBrain',
  description: 'BotBrain - Robotics Command and Control System',
  openGraph: {
    title: 'BotBrain',
    description: 'BotBrain - Robotics Command and Control System',
    type: 'website',
  },
  twitter: {
    card: 'summary',
    title: 'BotBrain',
    description: 'BotBrain - Robotics Command and Control System',
  },
};

export const viewport: Viewport = {
  width: 'device-width',
  initialScale: 1,
  maximumScale: 5,
  userScalable: true,
};

export default function RootLayout({
  children,
}: Readonly<{
  children: React.ReactNode;
}>) {
  return (
    <html lang="en" className="h-full">
      <body
        className={`${workSans.variable} ${workSans.className} antialiased h-full m-0 p-0`}
      >
        <SupabaseProvider>
          <ThemeProvider>
            <SpeedModeProvider>
              <NotificationsProvider>
                <LanguageProvider>
                  <RobotConnectionProvider>
                  <RobotProfileProvider>
                    <RobotCustomModeProvider>
                      <ActiveMissionProvider>
                        <NavigationTargetsProvider>
                          <ConfirmationDialogProvider>
                            <HeaderProvider>
                              {children}
                              <PopupsContainer />
                            </HeaderProvider>
                          </ConfirmationDialogProvider>
                        </NavigationTargetsProvider>
                      </ActiveMissionProvider>
                    </RobotCustomModeProvider>
                  </RobotProfileProvider>
                  </RobotConnectionProvider>
                </LanguageProvider>
              </NotificationsProvider>
            </SpeedModeProvider>
          </ThemeProvider>
        </SupabaseProvider>
        <SpeedInsights />
      </body>
    </html>
  );
}
