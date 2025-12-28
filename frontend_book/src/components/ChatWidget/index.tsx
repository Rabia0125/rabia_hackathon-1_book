/**
 * ChatWidget Component
 *
 * Interactive chat interface for querying the RAG chatbot.
 * Allows users to ask questions about the Physical AI & Robotics book content.
 */

import React, { useState } from 'react';
import { sendChatQuery, ChatResponse } from './api';
import styles from './ChatWidget.module.css';

interface ChatWidgetProps {
  /** Optional pre-selected text from the page */
  selectedText?: string;
  /** Optional module filter to scope queries */
  moduleFilter?: 'intro' | 'ros2' | 'simulation' | 'isaac' | 'vla';
}

export const ChatWidget: React.FC<ChatWidgetProps> = ({ selectedText, moduleFilter }) => {
  // State management (T048)
  const [query, setQuery] = useState<string>('');
  const [loading, setLoading] = useState<boolean>(false);
  const [response, setResponse] = useState<ChatResponse | null>(null);
  const [error, setError] = useState<string | null>(null);

  // Handle form submission (T047, T049)
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
      // Send query to API (T049)
      const result = await sendChatQuery({
        query: query.trim(),
        selected_text: selectedText,
        module_filter: moduleFilter,
        top_k: 5,
      });

      setResponse(result);
    } catch (err) {
      // Error handling (T051)
      if (err instanceof Error) {
        setError(err.message);
      } else {
        setError('An unexpected error occurred. Please try again.');
      }
    } finally {
      setLoading(false);
    }
  };

  // Handle input change
  const handleQueryChange = (e: React.ChangeEvent<HTMLTextAreaElement>) => {
    setQuery(e.target.value);
  };

  return (
    <div className={styles.chatWidget}>
      <div className={styles.header}>
        <h3>Ask About the Book</h3>
        {selectedText && (
          <p className={styles.contextNote}>
            Context: "{selectedText.substring(0, 50)}..."
          </p>
        )}
      </div>

      {/* Query input form (T047) */}
      <form onSubmit={handleSubmit} className={styles.form}>
        <textarea
          className={styles.textarea}
          value={query}
          onChange={handleQueryChange}
          placeholder="Ask a question about the book content..."
          rows={4}
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
            {loading ? 'Searching...' : 'Ask'}
          </button>
        </div>
      </form>

      {/* Loading indicator (T052) */}
      {loading && (
        <div className={styles.loading}>
          <div className={styles.spinner}></div>
          <p>Searching the book for answers...</p>
        </div>
      )}

      {/* Error display (T051) */}
      {error && (
        <div className={styles.error}>
          <strong>Error:</strong> {error}
        </div>
      )}

      {/* Response rendering (T050) */}
      {response && !loading && (
        <div className={styles.response}>
          {/* Answer */}
          <div className={styles.answer}>
            <h4>Answer:</h4>
            <p>{response.answer}</p>
          </div>

          {/* Confidence indicator */}
          <div className={`${styles.confidence} ${styles[response.confidence]}`}>
            Confidence: <span className={styles.confidenceBadge}>{response.confidence}</span>
          </div>

          {/* Citations (T050) */}
          {response.citations.length > 0 && (
            <div className={styles.citations}>
              <h4>Sources:</h4>
              <ul className={styles.citationList}>
                {response.citations.map((citation, index) => (
                  <li key={index} className={styles.citation}>
                    <a href={citation.page_url} target="_blank" rel="noopener noreferrer">
                      {citation.page_title}
                    </a>
                    <span className={styles.module}>({citation.module_name})</span>
                  </li>
                ))}
              </ul>
            </div>
          )}

          {/* Timing metrics (T053 - optional) */}
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
  );
};

export default ChatWidget;
