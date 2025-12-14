<!-- Sync Impact Report:
Version change: 1.1.0 -> 2.0.0
Modified principles:
  - Spec-Driven Writing -> Agent-Driven Architecture
  - Technical Reliability -> MCP Integration & Validation
  - Beginner-Friendly + Professional Tone -> Production-Ready Implementation
  - Documentation Quality -> Agent Response Quality
  - Consistency -> System Consistency
Added sections: Agent Architecture, MCP Integration, Vector Database, Chat Frontend, Backend Architecture, Deployment Strategy, Output Rules, Quality Rules, Validation Requirements
Removed sections: Book Requirements, Technical Constraints (replaced with new sections)
Templates requiring updates:
  - .specify/templates/plan-template.md: ⚠ pending
  - .specify/templates/spec-template.md: ⚠ pending
  - .specify/templates/tasks-template.md: ⚠ pending
  - .specify/templates/commands/*.md: ⚠ pending
Follow-up TODOs: none
-->
# AI-Powered RAG Chatbot Constitution

## Core Principles

### I. Agent-Driven Architecture
All chatbot intelligence must run through OpenAI Agents SDK with Gemini 2.5 Flash as the LLM model. The agent must use the Responses API format for generating answers and support two modes: normal RAG mode (retrieve relevant chunks from Qdrant) and selected text mode (answer ONLY from selected text supplied by the user).

### II. MCP Integration & Validation
MCP tools must be used as skills for chunking markdown, creating embeddings, Qdrant upsert/search, and document selection & filtering. The agent must call these MCP tools via OpenAI Agents SDK tools API, following MCP context7 guidance for standardization and reuse.

### III. Production-Ready Implementation
All code generated must be self-contained, production-ready, and follow the project spec. Backend must be deployable on Hugging Face Spaces and frontend on Vercel. All environment variables must be configured for OpenAI, Gemini, and Qdrant.

### IV. Agent Response Quality
The agent must ALWAYS cite chunks when answering and is NOT allowed to hallucinate. If selected text is provided, the agent will perform retrieval only on the selected text using the same RAG mechanism, ensuring identical output formatting to normal RAG answers.

### V. System Consistency
All components must maintain consistency in behavior, formatting, and user experience. Both normal RAG mode and selected text mode must produce answers with identical formatting, citations, and clarity.

## Agent Architecture

- All chatbot intelligence runs through **OpenAI Agents SDK**
- LLM model used by the agent is always **Gemini 2.5 Flash**
- Agent uses the `Responses` API format for generating answers
- Agent supports two modes:
  a. Normal RAG mode (retrieve relevant chunks from Qdrant)
  b. Selected Text mode (answer ONLY from selected text supplied by the user)
- Agent implementation follows **MCP context7 guidance** for creating OpenAI Agents SDK agents, connecting to external LLMs, implementing tool calls for Qdrant vector retrieval, and handling selected text as context

## MCP Integration

- MCP tools used as skills for:
  - Chunking markdown
  - Creating embeddings
  - Qdrant upsert/search
  - Document selection & filtering
- Agent calls these MCP tools via OpenAI Agents SDK tools API
- Use **MCP context7** as reference for all skill implementations to ensure standardization and reuse

## Vector Database (Qdrant)

- Qdrant Cloud Free Tier will be used
- Retrieval done using the "function-tool" mechanism inside the agent
- All queries to Qdrant must be RAG-proper:
  - top_k retrieval
  - metadata return
  - similarity score

## Chat Frontend (ChatKit)

- Chatbot UI built using **ChatKit widget**
- ChatKit communicates only with FastAPI backend
- ChatKit supports:
  - selected-text-only mode (context override)
  - history-based chat threads
  - selected-text answers behave like normal RAG answers (format, citations, clarity)

## Backend Architecture (FastAPI → Hugging Face Deployment)

- FastAPI exposes:
  - `/session` — create a new agent session
  - `/chat` — send user message + optional selected text
  - `/rag` — internal service to retrieve Qdrant chunks via MCP
- Backend is fully deployable on **Hugging Face Spaces**
- All environment variables configured for OpenAI, Gemini, and Qdrant
- Backend logic follows MCP context7 instructions for:
  - Agent initialization
  - Tool integration
  - Selected text handling

## Deployment Strategy

- Backend (FastAPI) deployed on **Hugging Face Spaces**
- Frontend (Docusaurus + ChatKit widget) deployed on **Vercel**

## Output Rules

- The agent must ALWAYS cite chunks when answering
- The agent is NOT allowed to hallucinate
- If selected text is provided, the agent will perform retrieval only on the selected text using the same RAG mechanism, ensuring the answer format remains identical to normal RAG answers

## Quality Rules

- All code generated must be self-contained, production-ready, and follow the project spec
- All tasks must be broken down using Spec-Kit Plus workflow:
  - `/sp.specify`
  - `/sp.clarify`
  - `/sp.plan`
  - `/sp.tasks`
  - `/sp.implement`
- Use MCP context7 documentation at every step for guidance

## Validation Requirements

- At every step, Claude Code must validate:
  - Qdrant schema compatibility
  - working MCP skills
  - correct agent JSON schema
  - ChatKit frontend script stability
  - selected text handling correctness
  - Gemini 2.5 Flash LLM connectivity

## Governance

This constitution supersedes all other practices. Amendments require documentation, approval, and a migration plan. All pull requests and reviews must verify compliance with Agent Architecture, MCP Integration, Vector Database, Chat Frontend, Backend Architecture, and Quality Rules. Complexity must be justified.

### Success Criteria

- Agent runs through OpenAI Agents SDK with Gemini 2.5 Flash
- MCP tools properly integrated for all required operations
- Qdrant vector database properly configured and queried
- FastAPI backend deployed on Hugging Face Spaces
- ChatKit frontend deployed on Vercel
- Both RAG modes (normal and selected text) function correctly
- All code follows production-ready standards
- Spec-Kit Plus workflow followed end-to-end
- All validation requirements met

**Version**: 2.0.0 | **Ratified**: 2025-12-12 | **Last Amended**: 2025-12-12