'use client';

import { useRouter } from 'next/navigation';
import { Network, Home, Layout, ArrowRight, Sparkles } from 'lucide-react';

interface QuickAccessItem {
  icon: React.ReactNode;
  title: string;
  href: string;
  gradient: string;
  description: string;
  highlight?: boolean;
}

export default function CompactQuickAccess() {
  const router = useRouter();

  const items: QuickAccessItem[] = [
    {
      icon: <Home className="h-5 w-5" />,
      title: "Cockpit",
      href: "/cockpit",
      gradient: "from-violet-500 to-purple-600",
      description: "Control & monitor your robot",
      highlight: true
    },
    {
      icon: <Network className="h-5 w-5" />,
      title: "Fleet Manager",
      href: "/fleet",
      gradient: "from-blue-500 to-cyan-600",
      description: "Manage all your robots"
    },
    {
      icon: <Layout className="h-5 w-5" />,
      title: "My UI",
      href: "/my-ui",
      gradient: "from-green-500 to-emerald-600",
      description: "Customize your workspace"
    }
  ];

  return (
    <div className="grid grid-cols-1 sm:grid-cols-3 gap-4">
      {items.map((item, index) => (
        <button
          key={item.href}
          onClick={() => router.push(item.href)}
          className={`
            group relative overflow-hidden bg-white dark:bg-botbot-dark
            border-2 ${item.highlight ? 'border-violet-300 dark:border-violet-700' : 'border-gray-200 dark:border-gray-700'}
            rounded-2xl p-5 hover:shadow-xl hover:scale-[1.02]
            transition-all duration-300 hover:border-violet-400 dark:hover:border-violet-500
          `}
          style={{ animationDelay: `${index * 50}ms` }}
        >
          {/* Gradient overlay */}
          <div
            className={`absolute inset-0 bg-gradient-to-br ${item.gradient} opacity-0
              group-hover:opacity-10 transition-opacity duration-300`}
          />

          {/* Highlight indicator for Robot Home */}
          {item.highlight && (
            <div className="absolute top-3 right-3">
              <Sparkles className="h-4 w-4 text-violet-500 dark:text-violet-400 animate-pulse" />
            </div>
          )}

          <div className="relative flex items-center gap-4">
            {/* Icon with gradient background */}
            <div className={`
              relative p-3 rounded-xl bg-gradient-to-br ${item.gradient}
              text-white shadow-lg group-hover:scale-110 group-hover:rotate-3
              transition-all duration-300
            `}>
              {item.icon}
              {/* Glow effect on hover */}
              <div className={`
                absolute inset-0 rounded-xl bg-gradient-to-br ${item.gradient}
                blur-xl opacity-0 group-hover:opacity-50 transition-opacity duration-300
              `} />
            </div>

            {/* Content */}
            <div className="flex-1 text-left">
              <h3 className="text-base font-bold text-gray-900 dark:text-white mb-0.5
                group-hover:text-violet-600 dark:group-hover:text-violet-400 transition-colors">
                {item.title}
              </h3>
              <p className="text-sm text-gray-600 dark:text-gray-400">
                {item.description}
              </p>
            </div>

            {/* Arrow indicator */}
            <ArrowRight className={`
              h-5 w-5 text-gray-400 dark:text-gray-600
              opacity-0 group-hover:opacity-100
              transform translate-x-2 group-hover:translate-x-0
              transition-all duration-300
            `} />
          </div>

          {/* Bottom gradient line on hover */}
          <div className={`
            absolute bottom-0 left-0 right-0 h-0.5
            bg-gradient-to-r ${item.gradient}
            transform scale-x-0 group-hover:scale-x-100
            transition-transform duration-300 origin-left
          `} />
        </button>
      ))}
    </div>
  );
}