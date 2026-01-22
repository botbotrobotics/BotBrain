export interface ChatMessage {
  id: string;
  content: string;
  sender: 'user' | 'robot';
  timestamp: Date;
  status?: 'sending' | 'sent' | 'error';
}
