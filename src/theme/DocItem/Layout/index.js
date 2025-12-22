import React from 'react';
import clsx from 'clsx';
import { useDoc } from '@docusaurus/plugin-content-docs/client';
import DocItemPaginator from '@theme/DocItem/Paginator';
import DocVersionBanner from '@theme/DocVersionBanner';
import DocVersionBadge from '@theme/DocVersionBadge';
import DocItemFooter from '@theme/DocItem/Footer';
import DocItemContent from '@theme/DocItem/Content';
import DocBreadcrumbs from '@theme/DocBreadcrumbs';
import { useAuth } from '../../../auth/context/AuthContext';
import PersonalizeChapterButton from '../../../personalization/components/PersonalizeChapterButton';
import PersonalizedContentView from '../../../personalization/components/PersonalizedContentView';

import styles from './styles.module.css';

export default function DocItemLayout({ children }) {
  const { frontMatter, contentTitle, metadata } = useDoc();
  const { state: authState } = useAuth();

  // Get the current chapter ID from the metadata
  const chapterId = metadata.source?.replace(/\W/g, '_') || metadata.unversionedId || 'unknown';

  // State for personalization
  const [isPersonalized, setIsPersonalized] = React.useState(false);
  const [personalizedContent, setPersonalizedContent] = React.useState('');
  const [originalContent, setOriginalContent] = React.useState('');

  // Function to handle personalization
  const handlePersonalize = (chapterId, content) => {
    setPersonalizedContent(content);
    setIsPersonalized(true);
  };

  // Store original content when component mounts
  React.useEffect(() => {
    if (children && typeof children === 'string') {
      setOriginalContent(children);
    }
  }, [children]);

  return (
    <div className="row">
      <div className={clsx('col', !frontMatter.hide_table_of_contents && styles.docItemCol)}>
        <DocVersionBanner />
        <DocVersionBadge />
        <DocBreadcrumbs />
        <div className={styles.docItemContainer}>
          <article>
            {contentTitle && <h1 className="docTitle">{contentTitle}</h1>}
            <div className="markdown">
              <PersonalizedContentView
                chapterId={chapterId}
                originalContent={originalContent}
                personalizedContent={personalizedContent}
                isPersonalized={isPersonalized}
                onTogglePersonalization={() => setIsPersonalized(!isPersonalized)}
              />
            </div>
            <DocItemContent>
              {children}
            </DocItemContent>
            <footer>
              <PersonalizeChapterButton
                chapterId={chapterId}
                onPersonalize={handlePersonalize}
                currentContent={originalContent}
              />
              <DocItemFooter />
            </footer>
          </article>
          <DocItemPaginator />
        </div>
      </div>
    </div>
  );
}