# ChatWidget Integration Guide

This guide explains how to integrate the ChatWidget component into your Docusaurus pages.

## Quick Start

### Option 1: Import in a Specific Page

Add the ChatWidget to any MDX page:

```mdx
---
title: My Page
---

import ChatWidget from '@site/src/components/ChatWidget';

# My Page Content

<ChatWidget />

More content here...
```

### Option 2: Add to All Pages via Docusaurus Plugin

To add the ChatWidget to all pages automatically, you can use a Docusaurus theme wrapper.

**Step 1**: Create a theme wrapper file `src/theme/Root.tsx`:

```tsx
import React from 'react';
import ChatWidget from '@site/src/components/ChatWidget';

export default function Root({children}) {
  return (
    <>
      {children}
      <div style={{
        position: 'fixed',
        bottom: '20px',
        right: '20px',
        zIndex: 1000,
        maxWidth: '400px'
      }}>
        <ChatWidget />
      </div>
    </>
  );
}
```

**Step 2**: Configure Docusaurus to use the wrapper in `docusaurus.config.ts`:

```typescript
const config: Config = {
  // ... other config
  themes: [],
  plugins: [],
  // The Root wrapper is automatically detected by Docusaurus
};
```

### Option 3: Add to Sidebar or Navbar

**Navbar Integration** (`docusaurus.config.ts`):

```typescript
themeConfig: {
  navbar: {
    items: [
      // ... other navbar items
      {
        type: 'custom-ChatWidget',
        position: 'right',
      },
    ],
  },
}
```

Then create `src/theme/NavbarItem/CustomChatWidget.tsx`:

```tsx
import React, { useState } from 'react';
import ChatWidget from '@site/src/components/ChatWidget';

export default function CustomChatWidget() {
  const [isOpen, setIsOpen] = useState(false);

  return (
    <div>
      <button
        onClick={() => setIsOpen(!isOpen)}
        style={{ background: 'var(--ifm-color-primary)', color: 'white', padding: '8px 16px', borderRadius: '4px', border: 'none', cursor: 'pointer' }}
      >
        Ask AI
      </button>
      {isOpen && (
        <div style={{ position: 'absolute', top: '60px', right: '20px', width: '400px', zIndex: 9999 }}>
          <ChatWidget />
        </div>
      )}
    </div>
  );
}
```

## Configuration

### Environment Variables

Create a `.env.local` file in `frontend_book/`:

```bash
REACT_APP_API_URL=http://localhost:8000
```

For production:

```bash
REACT_APP_API_URL=https://your-api-domain.com
```

### Component Props

The ChatWidget component accepts optional props:

```tsx
interface ChatWidgetProps {
  /** Optional pre-selected text from the page */
  selectedText?: string;
  /** Optional module filter to scope queries */
  moduleFilter?: 'intro' | 'ros2' | 'simulation' | 'isaac' | 'vla';
}
```

**Example with selected text**:

```tsx
<ChatWidget selectedText={window.getSelection()?.toString()} />
```

**Example with module filter**:

```tsx
// On a ROS 2 page, filter queries to ros2 module
<ChatWidget moduleFilter="ros2" />
```

## Styling

The component uses CSS modules (`ChatWidget.module.css`) and follows Docusaurus conventions for theming. It automatically adapts to light/dark mode using Docusaurus CSS variables.

### Customizing Styles

To override styles, create a custom CSS file:

```css
/* src/css/custom.css */
.chatWidget {
  /* Custom styles */
  max-width: 600px;
  box-shadow: 0 4px 12px rgba(0, 0, 0, 0.15);
}
```

Import in `src/pages/index.tsx` or `docusaurus.config.ts`.

## Advanced Usage

### Detecting Selected Text

To enable contextual queries based on selected text:

```tsx
import React, { useState, useEffect } from 'react';
import ChatWidget from '@site/src/components/ChatWidget';

export default function PageWithSelection() {
  const [selectedText, setSelectedText] = useState<string>('');

  useEffect(() => {
    const handleSelection = () => {
      const selection = window.getSelection()?.toString();
      if (selection && selection.length > 0) {
        setSelectedText(selection);
      }
    };

    document.addEventListener('mouseup', handleSelection);
    return () => document.removeEventListener('mouseup', handleSelection);
  }, []);

  return (
    <div>
      <h1>My Content</h1>
      <p>Select any text on this page to ask questions about it.</p>
      <ChatWidget selectedText={selectedText} />
    </div>
  );
}
```

### Module-Specific Widgets

Create page-specific widgets that auto-filter by module:

```tsx
// src/components/ROS2ChatWidget.tsx
import React from 'react';
import ChatWidget from './ChatWidget';

export default function ROS2ChatWidget() {
  return <ChatWidget moduleFilter="ros2" />;
}
```

Then use in ROS 2 pages:

```mdx
import ROS2ChatWidget from '@site/src/components/ROS2ChatWidget';

<ROS2ChatWidget />
```

## Troubleshooting

### CORS Errors

If you see CORS errors in the browser console:

1. Verify backend `CORS_ORIGINS` includes your frontend URL
2. Restart both frontend and backend servers
3. Check browser dev tools → Network tab → response headers

### API Connection Issues

If the widget shows connection errors:

1. Verify backend is running: `curl http://localhost:8000/health`
2. Check `REACT_APP_API_URL` in `.env.local`
3. Verify no firewall blocking localhost:8000

### Styling Issues

If styles don't apply correctly:

1. Clear Docusaurus cache: `npm run clear`
2. Rebuild: `npm run build`
3. Check for CSS module import issues in browser console

## Production Deployment

### Backend Deployment

Deploy the FastAPI backend first (see `backend/README.md`).

### Frontend Deployment

Update `.env.local` (or platform environment variables):

```bash
REACT_APP_API_URL=https://your-production-api.com
```

Build and deploy:

```bash
npm run build
# Deploy build/ directory to your hosting platform
```

### Vercel Deployment

For Vercel, add environment variable in dashboard:
- Key: `REACT_APP_API_URL`
- Value: `https://your-production-api.com`

## Examples

### Minimal Example

```tsx
import ChatWidget from '@site/src/components/ChatWidget';

<ChatWidget />
```

### With Selected Text

```tsx
<ChatWidget selectedText="ros2 run demo_nodes_cpp talker" />
```

### Module Filtered

```tsx
<ChatWidget moduleFilter="simulation" />
```

### Full Example with Selection Detection

```tsx
import React, { useState, useEffect } from 'react';
import ChatWidget from '@site/src/components/ChatWidget';

export default function SmartChatWidget() {
  const [selected, setSelected] = useState('');

  useEffect(() => {
    const handler = () => setSelected(window.getSelection()?.toString() || '');
    document.addEventListener('mouseup', handler);
    return () => document.removeEventListener('mouseup', handler);
  }, []);

  return <ChatWidget selectedText={selected} />;
}
```

## Support

For issues or questions:
- Check `backend/README.md` for API setup
- Review `frontend_book/.env.local.example` for configuration
- See `specs/007-fastapi-backend-integration/quickstart.md` for troubleshooting
