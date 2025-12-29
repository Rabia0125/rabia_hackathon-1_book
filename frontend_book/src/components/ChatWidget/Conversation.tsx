/**
 * Conversation-Style Chat Widget
 *
 * Modern conversation-style chat interface with glass morphism design,
 * chat history, smooth animations, and enhanced UX.
 */

import React, { useState, useRef, useEffect } from 'react';
import { sendChatQuery, ChatResponse } from './api';
import ChatBubble from './ChatBubble';
import styles from './ChatWidget.module.css';

interface ChatMessage {
  role: 'user' | 'assistant';
  content: string;
  timestamp: Date;
}

interface ChatWidgetProps {
  /** Optional pre-selected text from page */
  selectedText?: string;
  /** Optional module filter to scope queries */
  moduleFilter?: 'intro' | 'ros2' | 'simulation' | 'isaac' | 'vla';
}

export const Conversation: React.FC<ChatWidgetProps> = ({ selectedText, moduleFilter }) => {
  // State management
  const [query, setQuery] = useState<string>('');
  const [loading, setLoading] = useState<boolean>(false);
  const [response, setResponse] = useState<ChatResponse | null>(null);
  const [error, setError] = useState<string | null>(null);
  const [chatHistory, setChatHistory] = useState<ChatMessage[]>([]);
  const [showHistory, setShowHistory] = useState<boolean>(true); // Show history by default
  const [showDetails, setShowDetails] = useState<boolean>(false); // Show response details (collapsed by default)
  const scrollRef = React.useRef<HTMLDivElement>(null);
  const inputRef = React.useRef<HTMLTextAreaElement>(null);
  const [copied, setCopied] = useState<boolean>(false);

  // Auto-scroll to bottom when messages change
  useEffect(() => {
    if (scrollRef.current) {
      // Use requestAnimationFrame for smoother scrolling
      requestAnimationFrame(() => {
        if (scrollRef.current) {
          scrollRef.current.scrollTop = scrollRef.current.scrollHeight;
        }
      });
    }
  }, [chatHistory, loading, error]); // Re-run on all message changes

  // Focus input when conversation starts
  useEffect(() => {
    if (inputRef.current && chatHistory.length === 0) {
      inputRef.current.focus();
    }
  }, []);

  // Copy answer to clipboard
  const copyToClipboard = async () => {
    if (response?.answer) {
      try {
        await navigator.clipboard.writeText(response.answer);
        setCopied(true);
        setTimeout(() => setCopied(false), 2000);
      } catch (err) {
        console.error('Failed to copy:', err);
      }
    }
  };

  // Clear chat history
  const clearChatHistory = () => {
    setChatHistory([]);
    setShowHistory(true);
    setResponse(null);
    setError(null);
    // Focus input after clearing
    setTimeout(() => {
      if (inputRef.current) {
        inputRef.current.focus();
      }
    }, 100);
  };

  // Handle form submission
  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();

    // Clear previous state
    setError(null);
    setResponse(null);

    // Validate query
    if (!query.trim()) {
      setError('Please enter a question');
      return;
    }

    setLoading(true);

    try {
      // Send query to API
      const result = await sendChatQuery({
        query: query.trim(),
        selected_text: selectedText,
        module_filter: moduleFilter,
        top_k: 5,
      });

      // Add user message to history
      const userMessage: ChatMessage = {
        role: 'user',
        content: query.trim(),
        timestamp: new Date(),
      };
      setChatHistory(prev => [...prev, userMessage]);

      setResponse(result);

      // Add assistant response to history
      const assistantMessage: ChatMessage = {
        role: 'assistant',
        content: result.answer,
        timestamp: new Date(),
      };
      setChatHistory(prev => [...prev, assistantMessage]);

      // Clear input after submission and focus
      setQuery('');
      setTimeout(() => {
        if (inputRef.current) {
          inputRef.current.focus();
        }
      }, 100);

    } catch (err) {
      setError(err instanceof Error ? err.message : 'An unexpected error occurred. Please try again.');
    } finally {
      setLoading(false);
    }
  };

  // Handle input change
  const handleQueryChange = (e: React.ChangeEvent<HTMLTextAreaElement>) => {
    setQuery(e.target.value);
  };

  // Handle keyboard shortcuts
  const handleKeyDown = (e: React.KeyboardEvent<HTMLTextAreaElement>) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      if (query.trim() && !loading) {
        handleSubmit(e as any);
      }
    }
  };

  // Get last assistant message for response details
  const lastAssistantMessage = chatHistory.filter(m => m.role === 'assistant').pop();

  return (
    <div className={styles.conversationContainer}>
      {/* Header */}
      <div className={styles.conversationHeader}>
        <div className={styles.headerContent}>
          <h3>💬 Ask About the Book</h3>
          {selectedText && (
            <p className={styles.contextNote}>
              Context: "{selectedText.substring(0, 50)}..."
            </p>
          )}
        </div>
        <div className={styles.headerActions}>
          <button onClick={() => setShowHistory(!showHistory)} className={styles.historyToggle}>
            {showHistory ? '📋 Hide Chat' : '📜 Show Chat'}
          </button>
          {chatHistory.length > 0 && (
            <button onClick={clearChatHistory} className={styles.clearButton}>
              🗑️ Clear
            </button>
          )}
        </div>
      </div>

      {/* Conversation Messages Area (Scrollable) */}
      {showHistory && (
        <div className={styles.conversationMessages} ref={scrollRef}>
          {chatHistory.length === 0 && !loading && !response ? (
            <div className={styles.welcomeMessage}>
              <p>👋 Welcome! Ask me anything about the Physical AI & Robotics book.</p>
              <p className={styles.suggestion}>Try asking about ROS 2, Gazebo, NVIDIA Isaac, or VLA models.</p>
            </div>
          ) : (
            chatHistory.map((msg, index) => (
              <ChatBubble
                key={index}
                content={msg.content}
                isUser={msg.role === 'user'}
                timestamp={msg.timestamp}
              />
            ))
          )}

          {/* Loading indicator */}
          {loading && (
            <div className={styles.loadingBubble}>
              <div className={styles.spinner}></div>
              <span className={styles.loadingText}>Thinking...</span>
            </div>
          )}

          {/* Error display */}
          {error && (
            <div className={styles.errorBubble}>
              <strong>⚠️ Error:</strong> {error}
            </div>
          )}
        </div>
      )}

      {/* Response details (citations, metrics) - Only show if history is visible */}
      {showHistory && response && lastAssistantMessage && (
        <div className={styles.responseDetailsContainer}>
          {/* Toggle button for response details */}
          <button
            onClick={() => setShowDetails(!showDetails)}
            className={styles.detailsToggle}
          >
            {showDetails ? '📋 Hide Details' : '📚 Show Sources & Details'}
          </button>

          {/* Collapsible response details */}
          {showDetails && (
            <div className={styles.responseDetails}>
              {response.citations.length > 0 && (
                <div className={styles.citations}>
                  <div className={styles.citationsHeader}>
                    <span>📚 Sources</span>
                    {copied && <span className={styles.copyFeedback}>✓ Copied!</span>}
                  </div>
                  <div className={styles.citationList}>
                    {response.citations.map((citation, index) => (
                      <a
                        key={index}
                        href={citation.page_url}
                        target="_blank"
                        rel="noopener noreferrer"
                        className={styles.citation}
                      >
                        {citation.page_title}
                        <span className={styles.module}> ({citation.module_name})</span>
                      </a>
                    ))}
                  </div>
                </div>
              )}

              {/* Confidence indicator */}
              <div className={`${styles.confidence} ${styles[response.confidence]}`}>
                Confidence: <span className={styles.confidenceBadge}>{response.confidence}</span>
              </div>

              {/* Copy button */}
              <button onClick={copyToClipboard} className={styles.copyButton}>
                📋 Copy Answer
              </button>

              {/* Timing metrics */}
              <div className={styles.metrics}>
                <small>
                  Retrieved in {response.retrieval_time_ms.toFixed(0)}ms,
                  generated in {response.generation_time_ms.toFixed(0)}ms,
                  total {response.total_time_ms.toFixed(0)}ms
                </small>
              </div>
            </div>
          )}
        </div>
      )}

      {/* Input Form */}
      <div className={styles.inputArea}>
        <form onSubmit={handleSubmit} className={styles.form}>
          <textarea
            ref={inputRef}
            className={styles.textarea}
            value={query}
            onChange={handleQueryChange}
            onKeyDown={handleKeyDown}
            placeholder="Type your question here... (Press Enter to send, Shift+Enter for new line)"
            rows={3}
            disabled={loading}
            maxLength={2000}
          />
          <div className={styles.formFooter}>
            <span className={styles.charCount}>
              {query.length} / 2000
            </span>
            <button
              type="submit"
              className={styles.submitButton}
              disabled={loading || !query.trim()}
            >
              {loading ? '🔄 Sending...' : '🚀 Send'}
            </button>
          </div>
        </form>
      </div>
    </div>
  );
};

export default Conversation;
