'use client';

import { ChatMessage } from '@/interfaces/ChatMessage';
import {
  Bot,
  ChevronDown,
  ChevronRight,
  Loader2,
  ArrowRight,
  User,
} from 'lucide-react';
import React, { useEffect, useRef, useState } from 'react';
import InputField from './ui/input-field';
import { useRobotConnection } from '../contexts/RobotConnectionContext';
import RobotOffline from './robot-offline';
import { useLanguage } from '@/contexts/LanguageContext';
import useRobotActionDispatcher from '@/hooks/ros/useRobotActionDispatcher';
import { auditLogger } from '@/utils/audit-logger';

export default function ChatTTS() {
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
      messagesScrollableAreaRef.current.scrollTo({
        top: messagesScrollableAreaRef.current.scrollHeight,
        behavior: 'smooth',
      });
    }
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

    // Log TTS generation
    auditLogger.logTTSGenerated(
      content,
      connection.connectedRobot?.id,
      connection.connectedRobot?.name
    );

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
        moveChatToBottom();
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
            ? '❌ TTS service not available via rosbridge. Please restart rosbridge_server after ensuring the Talk service is built in your ROS2 workspace.'
            : `❌ Could not send to TTS: ${error || 'Unknown error'}`,
          sender: 'robot',
          timestamp: new Date(),
        };
        setMessages((prev) => [...prev, errorResponse]);
        moveChatToBottom();
      },
    });
  };

  return (
    <div className="h-full w-full flex flex-col">
      {/* Messages area - takes available space */}
      <div className="flex-1 min-h-0 overflow-hidden">
        {connection.online ? (
          <div
            className="h-full overflow-y-auto p-4 space-y-4"
            ref={messagesScrollableAreaRef}
          >
            {messages.map((message) => (
              <div
                key={message.id}
                className={`flex items-start gap-3 ${
                  message.sender === 'user' ? 'flex-row-reverse' : ''
                }`}
              >
                <div
                  className={`w-8 h-8 rounded-full flex items-center justify-center flex-shrink-0 ${
                    message.sender === 'user'
                      ? 'bg-[#821db7] dark:bg-botbot-dark'
                      : 'bg-[#f0d5ff] dark:bg-botbot-darker'
                  }`}
                >
                  {message.sender === 'user' ? (
                    <User className="w-4 h-4 text-white" />
                  ) : (
                    <Bot className="w-4 h-4 text-[#821db7] dark:text-white" />
                  )}
                </div>
                <div
                  className={`max-w-[80%] rounded-lg p-3 ${
                    message.sender === 'user'
                      ? 'bg-[#821db7] dark:bg-botbot-dark text-white'
                      : 'bg-[#f0d5ff] dark:bg-botbot-darker'
                  }`}
                >
                  <p className="text-sm">{message.content}</p>
                  <div className="flex items-center gap-2 mt-1">
                    <span className="text-xs opacity-75">
                      {message.timestamp.toLocaleTimeString()}
                    </span>
                    {message.sender === 'user' && message.status && (
                      <span className="text-xs">
                        {message.status === 'sending' ? (
                          <Loader2 className="w-3 h-3 animate-spin" />
                        ) : message.status === 'sent' ? (
                          <ChevronRight className="w-3 h-3" />
                        ) : (
                          <ChevronDown className="w-3 h-3" />
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
      <div className="border-t border-gray-200 dark:border-gray-700 p-4">
        <form
          onSubmit={(e) => {
            e.preventDefault();
            if (message.trim() && connection.online) {
              sendMessage(message);
            }
          }}
          className="flex gap-2"
        >
          <InputField
            placeholder={connection.online ? t('chat', 'typeMessage') : 'Robot offline'}
            className="flex-1 bg-white dark:bg-botbot-darker/50"
            value={message}
            name="message"
            disabled={disableInput}
            onChange={(e) => setMessage(e.target.value)}
          />
          <button
            type="submit"
            disabled={disableInput || !message.trim()}
            className="px-4 py-2 bg-primary hover:bg-primary/90 dark:bg-botbot-accent dark:hover:bg-botbot-accent/90 text-white rounded-lg transition-colors disabled:opacity-50 disabled:cursor-not-allowed"
          >
            <ArrowRight className="w-5 h-5" />
          </button>
        </form>
      </div>
    </div>
  );
}