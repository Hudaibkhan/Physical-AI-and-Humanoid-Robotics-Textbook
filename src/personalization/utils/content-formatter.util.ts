/**
 * Utility functions to ensure personalized content maintains original citations and formatting
 */

/**
 * Preserves citations in the format [source_id] when personalizing content
 */
export function preserveCitations(content: string): string {
  // Regular expression to match citation patterns like [source_id], [1], [Smith2020], etc.
  const citationRegex = /\[([^\]]+)\]/g;

  // Extract all citations to preserve them
  const citations = content.match(citationRegex) || [];

  // In a real implementation, we would ensure these citations are maintained
  // in the personalized content by passing them through to the agent
  console.log(`Found ${citations.length} citations to preserve:`, citations);

  return content;
}

/**
 * Preserves code blocks and their formatting during personalization
 */
export function preserveCodeBlocks(content: string): string {
  // Regular expression to match code blocks (both ``` and ~~~)
  const codeBlockRegex = /(```[\s\S]*?```|~~~[\s\S]*?~~~)/g;

  const codeBlocks = content.match(codeBlockRegex) || [];
  console.log(`Found ${codeBlocks.length} code blocks to preserve`);

  return content;
}

/**
 * Preserves headings structure during personalization
 */
export function preserveHeadings(content: string): string {
  // Regular expression to match markdown headings
  const headingRegex = /^(#{1,6})\s+(.*)$/gm;

  const headings = [];
  let match;
  while ((match = headingRegex.exec(content)) !== null) {
    headings.push({
      level: match[1].length,
      text: match[2]
    });
  }

  console.log(`Found ${headings.length} headings to preserve`);

  return content;
}

/**
 * Preserves list formatting during personalization
 */
export function preserveLists(content: string): string {
  // Regular expressions to match ordered and unordered lists
  const orderedListRegex = /^\d+\.\s+/gm;
  const unorderedListRegex = /^[\-\*\+]\s+/gm;

  const orderedListItems = content.match(orderedListRegex) || [];
  const unorderedListItems = content.match(unorderedListRegex) || [];

  console.log(`Found ${orderedListItems.length} ordered list items and ${unorderedListItems.length} unordered list items to preserve`);

  return content;
}

/**
 * Validates that personalized content maintains required formatting
 */
export function validateContentFormatting(originalContent: string, personalizedContent: string): boolean {
  // Check if citations are preserved
  const originalCitations = originalContent.match(/\[([^\]]+)\]/g) || [];
  const personalizedCitations = personalizedContent.match(/\[([^\]]+)\]/g) || [];

  // Basic validation: ensure all original citations appear in personalized content
  const allCitationsPreserved = originalCitations.every(citation =>
    personalizedCitations.includes(citation)
  );

  if (!allCitationsPreserved) {
    console.warn('Warning: Some citations may not be preserved in personalized content');
  }

  // Check if code blocks are preserved
  const originalCodeBlocks = originalContent.match(/(```[\s\S]*?```|~~~[\s\S]*?~~~)/g) || [];
  const personalizedCodeBlocks = personalizedContent.match(/(```[\s\S]*?```|~~~[\s\S]*?~~~)/g) || [];

  const allCodeBlocksPreserved = originalCodeBlocks.length === personalizedCodeBlocks.length;

  if (!allCodeBlocksPreserved) {
    console.warn('Warning: Code blocks may not be preserved in personalized content');
  }

  return allCitationsPreserved && allCodeBlocksPreserved;
}

/**
 * Processes content to ensure formatting is maintained during personalization
 */
export function processContentForPersonalization(content: string): string {
  // Preserve different content elements before sending to the agent
  const contentWithPreservedElements = content;

  // In a real implementation, this would tag or mark elements that need to be preserved
  // so the agent can maintain them during personalization

  return contentWithPreservedElements;
}

/**
 * Restores preserved formatting elements after personalization
 */
export function restoreContentFormatting(personalizedContent: string, originalContent: string): string {
  // In a real implementation, this would restore any formatting elements
  // that were preserved before sending to the agent

  // For now, we just return the personalized content
  return personalizedContent;
}