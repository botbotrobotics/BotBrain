'use client';

import React, { useEffect, useRef } from 'react';
import { useLanguage } from '@/contexts/LanguageContext';

interface LegalModalProps {
  isOpen: boolean;
  onClose: () => void;
  title: string;
  content: string;
  lastUpdated: string;
}

export default function LegalModal({
  isOpen,
  onClose,
  title,
  content,
  lastUpdated,
}: LegalModalProps) {
  const { t } = useLanguage();
  const modalRef = useRef<HTMLDivElement>(null);
  const contentRef = useRef<HTMLDivElement>(null);

  // Close on escape key
  useEffect(() => {
    const handleKeyDown = (e: KeyboardEvent) => {
      if (e.key === 'Escape' && isOpen) {
        onClose();
      }
    };

    if (isOpen) {
      document.addEventListener('keydown', handleKeyDown);
      document.body.style.overflow = 'hidden';
    }

    return () => {
      document.removeEventListener('keydown', handleKeyDown);
      document.body.style.overflow = 'unset';
    };
  }, [isOpen, onClose]);

  // Reset scroll position when modal opens
  useEffect(() => {
    if (isOpen && contentRef.current) {
      contentRef.current.scrollTop = 0;
    }
  }, [isOpen]);

  if (!isOpen) return null;

  // Parse content into formatted sections
  const renderContent = (text: string) => {
    const lines = text.trim().split('\n');
    const elements: React.ReactElement[] = [];
    let key = 0;

    for (let i = 0; i < lines.length; i++) {
      const line = lines[i];
      const trimmedLine = line.trim();

      // Skip empty lines at the start
      if (key === 0 && !trimmedLine) continue;

      // Main title (# )
      if (trimmedLine.startsWith('# ')) {
        elements.push(
          <h1 key={key++} className="text-2xl font-bold text-gray-900 dark:text-white mb-4">
            {trimmedLine.slice(2)}
          </h1>
        );
      }
      // Section headers (## )
      else if (trimmedLine.startsWith('## ')) {
        elements.push(
          <h2 key={key++} className="text-xl font-semibold text-gray-800 dark:text-gray-100 mt-6 mb-3 border-b border-gray-200 dark:border-gray-700 pb-2">
            {trimmedLine.slice(3)}
          </h2>
        );
      }
      // Subsection headers (### )
      else if (trimmedLine.startsWith('### ')) {
        elements.push(
          <h3 key={key++} className="text-lg font-medium text-gray-700 dark:text-gray-200 mt-4 mb-2">
            {trimmedLine.slice(4)}
          </h3>
        );
      }
      // Subsubsection headers (#### )
      else if (trimmedLine.startsWith('#### ')) {
        elements.push(
          <h4 key={key++} className="text-base font-medium text-gray-700 dark:text-gray-300 mt-3 mb-2">
            {trimmedLine.slice(5)}
          </h4>
        );
      }
      // Bold text line (**text**)
      else if (trimmedLine.startsWith('**') && trimmedLine.endsWith('**')) {
        elements.push(
          <p key={key++} className="font-semibold text-gray-800 dark:text-gray-200 my-2">
            {trimmedLine.slice(2, -2)}
          </p>
        );
      }
      // List items (- or *)
      else if (trimmedLine.startsWith('- ') || trimmedLine.startsWith('* ')) {
        elements.push(
          <li key={key++} className="text-gray-600 dark:text-gray-300 ml-4 list-disc">
            {formatInlineText(trimmedLine.slice(2))}
          </li>
        );
      }
      // Table header row
      else if (trimmedLine.startsWith('|') && trimmedLine.includes('|')) {
        const cells = trimmedLine.split('|').filter(c => c.trim());
        // Check if next line is separator
        if (lines[i + 1] && lines[i + 1].includes('---')) {
          elements.push(
            <div key={key++} className="overflow-x-auto my-4">
              <table className="min-w-full text-sm">
                <thead>
                  <tr className="border-b border-gray-300 dark:border-gray-600">
                    {cells.map((cell, idx) => (
                      <th key={idx} className="text-left py-2 px-3 font-semibold text-gray-700 dark:text-gray-200">
                        {cell.trim()}
                      </th>
                    ))}
                  </tr>
                </thead>
                <tbody>
                  {renderTableRows(lines.slice(i + 2), key)}
                </tbody>
              </table>
            </div>
          );
          // Skip to end of table
          while (i < lines.length - 1 && lines[i + 1].includes('|')) {
            i++;
          }
        }
      }
      // Regular paragraph
      else if (trimmedLine) {
        elements.push(
          <p key={key++} className="text-gray-600 dark:text-gray-300 my-2 leading-relaxed">
            {formatInlineText(trimmedLine)}
          </p>
        );
      }
      // Empty line (paragraph break)
      else {
        elements.push(<div key={key++} className="h-2" />);
      }
    }

    return elements;
  };

  // Format inline text (bold, links)
  const formatInlineText = (text: string): React.ReactNode => {
    // Handle bold text
    const parts = text.split(/(\*\*[^*]+\*\*)/g);
    return parts.map((part, idx) => {
      if (part.startsWith('**') && part.endsWith('**')) {
        return <strong key={idx} className="font-semibold text-gray-800 dark:text-gray-100">{part.slice(2, -2)}</strong>;
      }
      // Handle links
      const linkRegex = /\[([^\]]+)\]\(([^)]+)\)/g;
      const linkParts: React.ReactNode[] = [];
      let lastIndex = 0;
      let match;

      while ((match = linkRegex.exec(part)) !== null) {
        if (match.index > lastIndex) {
          linkParts.push(part.slice(lastIndex, match.index));
        }
        linkParts.push(
          <a
            key={`link-${idx}-${match.index}`}
            href={match[2]}
            target="_blank"
            rel="noopener noreferrer"
            className="text-botbot-purple hover:underline"
          >
            {match[1]}
          </a>
        );
        lastIndex = match.index + match[0].length;
      }

      if (linkParts.length > 0) {
        if (lastIndex < part.length) {
          linkParts.push(part.slice(lastIndex));
        }
        return <span key={idx}>{linkParts}</span>;
      }

      return part;
    });
  };

  // Render table body rows
  const renderTableRows = (lines: string[], startKey: number) => {
    const rows: React.ReactElement[] = [];
    let key = startKey;

    for (const line of lines) {
      const trimmed = line.trim();
      if (!trimmed.startsWith('|')) break;
      if (trimmed.includes('---')) continue;

      const cells = trimmed.split('|').filter(c => c.trim());
      rows.push(
        <tr key={key++} className="border-b border-gray-200 dark:border-gray-700">
          {cells.map((cell, idx) => (
            <td key={idx} className="py-2 px-3 text-gray-600 dark:text-gray-300">
              {cell.trim()}
            </td>
          ))}
        </tr>
      );
    }

    return rows;
  };

  return (
    <div className="fixed inset-0 z-50 flex items-center justify-center p-4">
      {/* Backdrop */}
      <div
        className="absolute inset-0 bg-black/60 backdrop-blur-sm animate-fadeIn"
        onClick={onClose}
      />

      {/* Modal */}
      <div
        ref={modalRef}
        className="relative w-full max-w-3xl max-h-[90vh] bg-white dark:bg-botbot-dark rounded-2xl shadow-2xl border border-gray-200 dark:border-botbot-purple/20 animate-fadeIn overflow-hidden"
      >
        {/* Header */}
        <div className="sticky top-0 z-10 flex items-center justify-between px-6 py-4 bg-white dark:bg-botbot-dark border-b border-gray-200 dark:border-gray-700">
          <div>
            <h2 className="text-xl font-bold text-gray-900 dark:text-white">
              {title}
            </h2>
            <p className="text-sm text-gray-500 dark:text-gray-400">
              {t('login', 'lastUpdated') || 'Last updated'}: {lastUpdated}
            </p>
          </div>
          <button
            onClick={onClose}
            className="p-2 rounded-full hover:bg-gray-100 dark:hover:bg-botbot-darker transition-colors"
            aria-label="Close"
          >
            <svg
              className="w-6 h-6 text-gray-500 dark:text-gray-400"
              fill="none"
              stroke="currentColor"
              viewBox="0 0 24 24"
            >
              <path
                strokeLinecap="round"
                strokeLinejoin="round"
                strokeWidth={2}
                d="M6 18L18 6M6 6l12 12"
              />
            </svg>
          </button>
        </div>

        {/* Content */}
        <div
          ref={contentRef}
          className="px-6 py-4 overflow-y-auto"
          style={{ maxHeight: 'calc(90vh - 140px)' }}
        >
          {renderContent(content)}
        </div>

        {/* Footer */}
        <div className="sticky bottom-0 z-10 flex justify-end px-6 py-4 bg-white dark:bg-botbot-dark border-t border-gray-200 dark:border-gray-700">
          <button
            onClick={onClose}
            className="px-6 py-2.5 bg-botbot-purple hover:bg-botbot-purple/90 text-white font-medium rounded-full transition-colors"
          >
            {t('login', 'iUnderstand') || 'I Understand'}
          </button>
        </div>
      </div>
    </div>
  );
}
