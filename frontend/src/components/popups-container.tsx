'use client';
import React, { useState } from 'react';
import Popup from './ui/popup';
import { useHeader } from '../contexts/HeaderContext';
import Chat from './chat';
import { ConfirmationDialog } from './ui/confirmation-dialog';

export default function PopupsContainer() {
  const { chatPopupOpen, setChatPopupOpen } = useHeader();
  const [ttsEnabled, setTtsEnabled] = useState(false);

  return (
    <>
      <Popup
        isOpen={chatPopupOpen}
        onClose={() => setChatPopupOpen(!chatPopupOpen)}
        title={
          <div className="flex items-center justify-between w-full">
            <span>Chat</span>
            <div className="flex items-center space-x-2 mr-4">
              <span className="text-sm text-gray-600 dark:text-gray-300">TTS</span>
              <label className="relative inline-flex items-center cursor-pointer">
                <input 
                  type="checkbox" 
                  className="sr-only peer" 
                  checked={ttsEnabled}
                  onChange={() => setTtsEnabled(!ttsEnabled)}
                />
                <div className="w-11 h-6 bg-gray-200 dark:bg-botbot-dark peer-focus:outline-none 
                               peer-focus:ring-2 peer-focus:ring-primary dark:peer-focus:ring-botbot-accent 
                               rounded-full peer peer-checked:after:translate-x-full 
                               peer-checked:after:border-white after:content-[''] after:absolute 
                               after:top-[2px] after:left-[2px] after:bg-white after:border-gray-300 
                               after:border after:rounded-full after:h-5 after:w-5 after:transition-all 
                               peer-checked:bg-primary dark:peer-checked:bg-botbot-accent">
                </div>
              </label>
            </div>
          </div>
        }
        className="w-[30rem] h-[35rem]"
        customContentClasses="w-full h-[32.25rem] flex flex-col justify-end"
      >
        <Chat ttsEnabled={ttsEnabled} />
      </Popup>
      <ConfirmationDialog />
    </>
  );
}
