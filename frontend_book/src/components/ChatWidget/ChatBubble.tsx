/**
 * ChatBubble Component
 *
 * Individual chat message bubble component for conversation-style UI.
 */

import React from 'react';
import ReactMarkdown from 'react-markdown';
import remarkGfm from 'remark-gfm';
import styles from './ChatBubble.module.css';

interface ChatBubbleProps {
  content: string;
  isUser: boolean;
  timestamp?: Date;
}

export const ChatBubble: React.FC<ChatBubbleProps> = ({ content, isUser, timestamp }) => {
  // Format timestamp
  const formatTime = (date: Date) => {
    return date.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' });
  };

  return (
    <div className={`${styles.chatBubble} ${isUser ? styles.userBubble : styles.assistantBubble}`}>
      <div className={styles.bubbleContent}>
        {isUser ? (
          // User messages as plain text (no markdown)
          <p className={styles.messageText}>{content}</p>
        ) : (
          // Assistant messages with markdown support
          <div className={styles.messageText}>
            <ReactMarkdown
              remarkPlugins={[remarkGfm]}
              components={{
                // Custom styling for markdown elements
                p: ({ children }) => <p style={{ margin: '0.5em 0' }}>{children}</p>,
                code: ({ children }) => (
                  <code style={{
                    background: 'rgba(0, 0, 0, 0.1)',
                    padding: '0.2em 0.4em',
                    borderRadius: '4px',
                    fontFamily: 'monospace',
                    fontSize: '0.9em'
                  }}>{children}</code>
                ),
                pre: ({ children }) => (
                  <pre style={{
                    background: 'rgba(0, 0, 0, 0.05)',
                    padding: '1em',
                    borderRadius: '8px',
                    overflowX: 'auto',
                    margin: '1em 0'
                  }}>{children}</pre>
                ),
                strong: ({ children }) => <strong style={{ fontWeight: '600' }}>{children}</strong>,
                em: ({ children }) => <em style={{ fontStyle: 'italic' }}>{children}</em>,
                ul: ({ children }) => <ul style={{ paddingLeft: '1.5em', margin: '0.5em 0' }}>{children}</ul>,
                ol: ({ children }) => <ol style={{ paddingLeft: '1.5em', margin: '0.5em 0' }}>{children}</ol>,
                li: ({ children }) => <li style={{ marginBottom: '0.3em' }}>{children}</li>,
                h1: ({ children }) => <h1 style={{ fontSize: '1.2em', margin: '0.8em 0 0.4em' }}>{children}</h1>,
                h2: ({ children }) => <h2 style={{ fontSize: '1.1em', margin: '0.7em 0 0.3em' }}>{children}</h2>,
                h3: ({ children }) => <h3 style={{ fontSize: '1.05em', margin: '0.6em 0 0.3em' }}>{children}</h3>,
                blockquote: ({ children }) => (
                  <blockquote style={{
                    borderLeft: '3px solid var(--ifm-color-primary)',
                    paddingLeft: '1em',
                    margin: '1em 0',
                    color: 'var(--ifm-color-emphasis-700)',
                    fontStyle: 'italic'
                  }}>{children}</blockquote>
                ),
                a: ({ children, href }) => (
                  <a
                    href={href}
                    target="_blank"
                    rel="noopener noreferrer"
                    style={{
                      color: isUser ? 'rgba(255,255,255,0.9)' : 'var(--ifm-color-primary)',
                      textDecoration: 'underline'
                    }}
                  >{children}</a>
                ),
              }}
            >
              {content}
            </ReactMarkdown>
          </div>
        )}
        {timestamp && (
          <span className={styles.timestamp}>{formatTime(timestamp)}</span>
        )}
      </div>
    </div>
  );
};

export default ChatBubble;
