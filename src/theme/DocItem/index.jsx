import React from 'react';
import { DocProvider } from '@docusaurus/plugin-content-docs/client';
import { HtmlClassNameProvider } from '@docusaurus/theme-common';
import DocItemMetadata from '@theme/DocItem/Metadata';
import DocItemLayout from '@theme/DocItem/Layout';

/**
 * DocItem Component
 *
 * This component follows the official Docusaurus pattern by wrapping
 * the content with DocProvider to enable useDoc() hook in child components.
 *
 * The personalization button is now handled in DocItem/Layout/index.js
 * which already has proper access to useDoc().
 *
 * Reference: specs/007-auth-integration-fix/research.md Section 2
 */
const DocItem = (props) => {
  const docHtmlClassName = `docs-doc-id-${props.content.metadata.id}`;
  const MDXComponent = props.content;

  return (
    <DocProvider content={props.content}>
      <HtmlClassNameProvider className={docHtmlClassName}>
        <DocItemMetadata />
        <DocItemLayout>
          <MDXComponent />
        </DocItemLayout>
      </HtmlClassNameProvider>
    </DocProvider>
  );
};

export default DocItem;