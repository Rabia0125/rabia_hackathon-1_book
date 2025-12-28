/**
 * API client for FastAPI backend
 *
 * Handles communication between the ChatWidget component and the backend API.
 * Provides TypeScript interfaces matching the backend Pydantic models.
 */

// API base URL from environment variable, fallback to localhost
const API_BASE_URL = process.env.REACT_APP_API_URL || 'http://localhost:8000';

/**
 * ChatRequest interface (matches backend ChatRequest model)
 */
export interface ChatRequest {
  query: string;
  selected_text?: string;
  module_filter?: 'intro' | 'ros2' | 'simulation' | 'isaac' | 'vla';
  top_k?: number;
}

/**
 * Citation interface (matches backend Citation model)
 */
export interface Citation {
  page_title: string;
  page_url: string;
  module_name: string;
}

/**
 * ChatResponse interface (matches backend ChatResponse model)
 */
export interface ChatResponse {
  answer: string;
  citations: Citation[];
  confidence: 'high' | 'low' | 'none';
  retrieval_time_ms: number;
  generation_time_ms: number;
  total_time_ms: number;
  error?: {
    code: string;
    message: string;
    suggestion?: string;
  };
}

/**
 * ErrorResponse interface for API errors
 */
export interface ErrorResponse {
  error: {
    code: string;
    message: string;
    suggestion?: string;
  };
}

/**
 * Send a chat query to the backend API
 *
 * @param request - ChatRequest with query and optional parameters
 * @returns Promise<ChatResponse> - The response with answer, citations, and metrics
 * @throws Error with user-friendly message if request fails
 */
export async function sendChatQuery(request: ChatRequest): Promise<ChatResponse> {
  try {
    const response = await fetch(`${API_BASE_URL}/chat`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify(request),
    });

    // Handle HTTP error responses
    if (!response.ok) {
      const errorData: ErrorResponse = await response.json();
      throw new Error(errorData.error.message || 'Failed to get response from the API');
    }

    const data: ChatResponse = await response.json();
    return data;
  } catch (error) {
    // Handle network errors or JSON parsing errors
    if (error instanceof Error) {
      throw error;
    }
    throw new Error('An unexpected error occurred. Please try again.');
  }
}

/**
 * Check API health status
 *
 * @returns Promise<boolean> - True if API is healthy
 */
export async function checkHealth(): Promise<boolean> {
  try {
    const response = await fetch(`${API_BASE_URL}/health`);
    return response.ok;
  } catch (error) {
    return false;
  }
}

/**
 * Check API readiness status
 *
 * @returns Promise<object> - Readiness status with checks
 */
export async function checkReadiness(): Promise<any> {
  try {
    const response = await fetch(`${API_BASE_URL}/ready`);
    if (response.ok) {
      return await response.json();
    }
    return null;
  } catch (error) {
    return null;
  }
}
