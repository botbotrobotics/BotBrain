'use client';

import { ChatMessage } from '@/interfaces/ChatMessage';
import {
  Bot,
  ChevronDown,
  ChevronRight,
  Loader2,
  ArrowRight,
  User,
  Volume2,
  Trash2,
} from 'lucide-react';
import React, { useEffect, useRef, useState, forwardRef, useImperativeHandle } from 'react';
import { useRobotConnection } from '../contexts/RobotConnectionContext';
import RobotOffline from './robot-offline';
import { useLanguage } from '@/contexts/LanguageContext';
import useRobotActionDispatcher from '@/hooks/ros/useRobotActionDispatcher';
import SoundboardTTSPresets from './soundboard-tts-presets';

export interface SoundboardTTSChatRef {
  sendMessage: (content: string) => void;
}

const SoundboardTTSChat = forwardRef<SoundboardTTSChatRef>((props, ref) => {
  const [message, setMessage] = useState('');
  const [messages, setMessages] = useState<ChatMessage[]>([]);
  const messagesEndRef = useRef<HTMLDivElement>(null);
  const messagesScrollableAreaRef = useRef<HTMLDivElement>(null);
  const [disableInput, setDisableInput] = useState(false);
  const { connection } = useRobotConnection();
  const dispatchAction = useRobotActionDispatcher();
  const { t } = useLanguage();

  // Update the input state based on connection
  useEffect(() => {
    setDisableInput(!connection.online);
  }, [connection.online]);

  // Scroll to bottom when new messages arrive
  useEffect(() => {
    moveChatToBottom();
  }, [messages]);

  const moveChatToBottom = () => {
    if (messagesScrollableAreaRef.current) {
      messagesScrollableAreaRef.current.scrollTop = messagesScrollableAreaRef.current.scrollHeight;
    }
  };

  const clearMessages = () => {
    setMessages([]);
  };

  const sendMessage = (content: string) => {
    const newMessage: ChatMessage = {
      id: Date.now().toString(),
      content,
      sender: 'user',
      timestamp: new Date(),
      status: 'sending',
    };

    setMessages((prev) => [...prev, newMessage]);
    setMessage(''); // Clear input after sending

    // Always send via TTS service
    dispatchAction({
      typeKey: 'talk',
      request: {
        text: content,
      },
      callback: (result: any) => {
        console.log('TTS service response:', result);
        // Update message status to sent
        setMessages((prev) =>
          prev.map((msg) =>
            msg.id === newMessage.id ? { ...msg, status: 'sent' } : msg
          )
        );
        
        // Add a system message indicating TTS was triggered
        const ttsResponse: ChatMessage = {
          id: (Date.now() + 1).toString(),
          content: '🔊 Text sent to speech system',
          sender: 'robot',
          timestamp: new Date(),
        };
        setMessages((prev) => [...prev, ttsResponse]);
      },
      failedCallback: (error?: string) => {
        console.error('TTS service error:', error);
        setMessages((prev) =>
          prev.map((msg) =>
            msg.id === newMessage.id ? { ...msg, status: 'error' } : msg
          )
        );
        
        // Add an error message to inform the user
        const errorResponse: ChatMessage = {
          id: (Date.now() + 1).toString(),
          content: error?.includes('Unable to import srv class') 
            ? '❌ TTS service not available. Please ensure the Talk service is built in your ROS2 workspace.'
            : `❌ Could not send to TTS: ${error || 'Unknown error'}`,
          sender: 'robot',
          timestamp: new Date(),
        };
        setMessages((prev) => [...prev, errorResponse]);
      },
    });
  };

  // Expose sendMessage method to parent component
  useImperativeHandle(ref, () => ({
    sendMessage
  }));

  return (
    <div className="h-full flex flex-col gap-3 overflow-hidden">
      {/* TTS Presets */}
      <div className="flex-shrink-0 rounded-default-border border border-gray-200 dark:border-black bg-white dark:bg-botbot-dark p-3 overflow-y-auto max-h-[40%]">
        <SoundboardTTSPresets onSendMessage={sendMessage} />
      </div>

      {/* TTS Chat */}
      <div className="flex-1 min-h-0 rounded-default-border border border-gray-200 dark:border-black bg-white dark:bg-botbot-dark flex flex-col overflow-hidden">
      {/* Header */}
      <div className="flex items-center justify-between px-4 py-3 border-b border-gray-200 dark:border-gray-700">
        <div className="flex items-center">
          <Volume2 className="w-5 h-5 mr-2 text-primary dark:text-botbot-accent" />
          <h2 className="text-base font-bold">{t('chat', 'tts')} {t('chat', 'title')}</h2>
        </div>
        <button
          onClick={clearMessages}
          className="p-1.5 hover:bg-gray-100 dark:hover:bg-botbot-darker rounded-lg transition-colors"
          title="Clear messages"
        >
          <Trash2 className="w-4 h-4" />
        </button>
      </div>

      {/* Messages area with fixed height and scroll */}
      <div className="flex-1 min-h-0 overflow-y-auto p-4" ref={messagesScrollableAreaRef}>
        {connection.online ? (
          <div className="space-y-3">
            {messages.length === 0 && (
              <div className="text-center text-gray-500 dark:text-gray-400 py-8">
                <Volume2 className="w-12 h-12 mx-auto mb-3 opacity-50" />
                <p className="text-sm">Send a message to trigger text-to-speech on the robot</p>
              </div>
            )}
            {messages.map((message) => (
              <div
                key={message.id}
                className={`flex items-start gap-2 ${
                  message.sender === 'user' ? 'flex-row-reverse' : ''
                }`}
              >
                <div
                  className={`w-7 h-7 rounded-full flex items-center justify-center flex-shrink-0 ${
                    message.sender === 'user'
                      ? 'bg-primary dark:bg-botbot-accent'
                      : 'bg-gray-200 dark:bg-botbot-darker'
                  }`}
                >
                  {message.sender === 'user' ? (
                    <User className="w-3.5 h-3.5 text-white" />
                  ) : (
                    <Bot className="w-3.5 h-3.5 text-gray-700 dark:text-gray-300" />
                  )}
                </div>
                <div
                  className={`max-w-[75%] rounded-lg px-3 py-2 ${
                    message.sender === 'user'
                      ? 'bg-primary text-white dark:bg-botbot-accent'
                      : 'bg-gray-100 dark:bg-botbot-darker text-gray-800 dark:text-gray-200'
                  }`}
                >
                  <p className="text-sm leading-relaxed">{message.content}</p>
                  <div className="flex items-center gap-1.5 mt-1">
                    <span className="text-xs opacity-70">
                      {message.timestamp.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })}
                    </span>
                    {message.sender === 'user' && message.status && (
                      <span className="text-xs opacity-70">
                        {message.status === 'sending' ? (
                          <Loader2 className="w-3 h-3 animate-spin" />
                        ) : message.status === 'sent' ? (
                          <ChevronRight className="w-3 h-3" />
                        ) : (
                          <ChevronDown className="w-3 h-3 text-red-300" />
                        )}
                      </span>
                    )}
                  </div>
                </div>
              </div>
            ))}
            <div ref={messagesEndRef} />
          </div>
        ) : (
          <div className="h-full flex items-center justify-center">
            <RobotOffline useBorder={false} />
          </div>
        )}
      </div>

      {/* Input area - fixed at bottom */}
      <div className="flex-shrink-0 border-t border-gray-200 dark:border-gray-700 p-3 bg-gray-50 dark:bg-botbot-darker/50">
        <form
          onSubmit={(e) => {
            e.preventDefault();
            if (message.trim() && connection.online) {
              sendMessage(message);
            }
          }}
          className="flex gap-2"
        >
          <input
            type="text"
            placeholder={connection.online ? "Type message for TTS..." : "Robot offline"}
            className="flex-1 px-3 py-2 bg-white dark:bg-botbot-dark border border-gray-300 dark:border-gray-600 rounded-lg focus:outline-none focus:ring-2 focus:ring-primary dark:focus:ring-botbot-accent text-sm"
            value={message}
            disabled={disableInput}
            onChange={(e) => setMessage(e.target.value)}
          />
          <button
            type="submit"
            disabled={disableInput || !message.trim()}
            className="px-4 py-2 bg-primary hover:bg-primary/90 dark:bg-botbot-accent dark:hover:bg-botbot-accent/90 text-white rounded-lg transition-colors disabled:opacity-50 disabled:cursor-not-allowed flex items-center justify-center"
          >
            <ArrowRight className="w-4 h-4" />
          </button>
        </form>
      </div>
      </div>
    </div>
  );
});

SoundboardTTSChat.displayName = 'SoundboardTTSChat';

export default SoundboardTTSChat;