<!--
=============================================================================
SYNC IMPACT REPORT
=============================================================================
Version Change: 0.0.0 → 1.0.0 (MAJOR - Initial constitution creation)

Modified Principles: N/A (new constitution)

Added Sections:
- Core Principles (6 principles)
- Technology Standards
- Development Workflow
- Governance

Removed Sections: None

Templates Requiring Updates:
- .specify/templates/plan-template.md: ✅ Compatible (Constitution Check section exists)
- .specify/templates/spec-template.md: ✅ Compatible (requirements align)
- .specify/templates/tasks-template.md: ✅ Compatible (task structure supports workflow)
- .specify/templates/phr-template.prompt.md: ✅ Compatible

Follow-up TODOs: None
=============================================================================
-->

# AI-Written Book with RAG Chatbot Constitution

## Core Principles

### I. Spec-Driven Reproducibility
Every feature, chapter, and component MUST begin with a specification document before implementation.
All workflows MUST be reproducible: given the same inputs and specs, any developer (human or AI) MUST
be able to recreate the output. Claude Code + Spec-Kit Plus provides the authoritative development workflow.

### II. Content Accuracy
The RAG chatbot MUST only answer questions using content retrieved from the book. No hallucinations
or fabrications are permitted. Every chatbot response MUST cite the specific section(s) from which
information was retrieved. Accuracy is non-negotiable: when the book doesn't contain relevant information,
the chatbot MUST acknowledge this limitation.

### III. Developer-Focused Writing
All book content MUST be written in clear, developer-focused prose. Technical concepts MUST be
explained with precision and practical examples. Avoid unnecessary jargon; when technical terms are
required, define them clearly. Code examples MUST be complete, runnable, and well-commented.

### IV. Retrieval Transparency
Users MUST be able to understand how the chatbot arrived at its answers. Retrieved sections MUST be
clearly cited with chapter/section references. The chatbot MUST support two query modes: full-book
queries (search entire content) and user-selected-text queries (search within highlighted text only).

### V. Public Reproducibility
The entire project MUST be publicly accessible and reproducible. All infrastructure choices (GitHub Pages,
Neon Postgres, Qdrant Cloud Free Tier) MUST support zero-cost or free-tier deployment. Setup
documentation MUST enable any developer to deploy their own instance from scratch.

### VI. Test-Driven Quality
All features MUST include testable acceptance criteria. RAG retrieval accuracy MUST be validated
against known question-answer pairs. The chatbot MUST be tested for edge cases: empty queries,
queries outside book scope, and adversarial inputs attempting to elicit hallucinations.

## Technology Standards

**Documentation Platform**: Docusaurus, deployed to GitHub Pages
- Static site generation for optimal performance
- Markdown-based authoring for AI-assisted writing

**RAG Chatbot Stack**:
- **Agent Framework**: OpenAI Agents/ChatKit
- **API Layer**: FastAPI (Python)
- **Vector Database**: Qdrant Cloud (Free Tier)
- **Relational Database**: Neon Serverless Postgres

**Query Modes**:
- Full-book queries: Search across all indexed book content
- Selected-text queries: Search limited to user-highlighted passages

**Constraints**:
- No paid infrastructure required for basic deployment
- All secrets managed via environment variables (never committed)
- Structured logging required for debugging retrieval issues

## Development Workflow

**Workflow Sequence**:
1. `/sp.specify` - Create feature specification from requirements
2. `/sp.clarify` - Resolve ambiguities via targeted questions
3. `/sp.plan` - Generate implementation plan with architecture decisions
4. `/sp.tasks` - Break down plan into testable, ordered tasks
5. `/sp.implement` - Execute tasks following TDD principles

**Content Workflow**:
1. Outline chapter in spec format
2. Generate draft via Claude Code
3. Review and iterate on content
4. Index content for RAG retrieval
5. Validate retrieval accuracy

**Quality Gates**:
- All specs MUST be reviewed before implementation
- All code changes MUST pass linting and type checks
- RAG accuracy MUST meet defined thresholds before deployment
- All commits MUST reference the relevant spec/task

## Governance

This constitution establishes the non-negotiable principles for the AI-Written Book with RAG Chatbot
project. All development decisions, code reviews, and feature implementations MUST comply with these
principles.

**Amendment Process**:
1. Proposed changes MUST be documented in a constitution amendment spec
2. Changes MUST include rationale and impact analysis
3. Version bump follows semantic versioning:
   - MAJOR: Principle removal or incompatible redefinition
   - MINOR: New principle or significant expansion
   - PATCH: Clarifications and non-semantic refinements

**Compliance**:
- All PRs MUST be verified against constitution principles
- Constitution violations MUST be resolved before merge
- Use `CLAUDE.md` for runtime development guidance

**Version**: 1.0.0 | **Ratified**: 2025-12-21 | **Last Amended**: 2025-12-21
