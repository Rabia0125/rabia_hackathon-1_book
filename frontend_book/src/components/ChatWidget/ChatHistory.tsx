/**
 * ChatHistory Component
 *
 * Scrollable chat conversation history component.
 */

import React from 'react';
import styles from './ChatHistory.module.css';

interface ChatHistoryProps {
  messages: ChatMessage[];
  onClear?: () => void;
}

export const ChatHistory: React.FC<ChatHistoryProps> = ({ messages, onClear }) => {
  const scrollRef = React.useRef<HTMLDivElement>(null);

  // Auto-scroll to bottom when new message arrives
  React.useEffect(() => {
    if (scrollRef.current && messages.length > 0) {
      scrollRef.current.scrollTop = scrollRef.current.scrollHeight;
    }
  }, [messages]);

  return (
    <div className={styles.chatHistory} ref={scrollRef}>
      <div className={styles.header}>
        <h4>Chat History</h4>
        {messages.length > 0 && onClear && (
          <button onClick={onClear} className={styles.clearButton}>
            🗑️ Clear Chat
          </button>
        )}
      </div>
      <div className={styles.messagesContainer}>
        {messages.length === 0 ? (
          <div className={styles.emptyState}>
            <p>Start a conversation to see messages here.</p>
          </div>
        ) : (
          messages.map((msg, index) => (
            <div key={index} className={`${styles.message} ${msg.isUser ? styles.userMessage : styles.assistantMessage}`}>
              {msg.content}
            </div>
          ))
        )}
      </div>
    </div>
  );
};

export default ChatHistory;
