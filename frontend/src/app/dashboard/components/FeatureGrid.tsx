'use client';

import { useRouter } from 'next/navigation';
import { Network, Home, Layout, ArrowUpRight } from 'lucide-react';

interface FeatureCardProps {
  icon: React.ReactNode;
  title: string;
  description: string;
  href: string;
  gradient: string;
  delay?: number;
  variant?: 'default' | 'glass' | 'neon' | 'gradient' | 'minimal';
  badge?: string;
  stats?: { label: string; value: string }[];
}

function FeatureCard({
  icon,
  title,
  description,
  href,
  gradient,
  delay = 0,
  variant = 'default',
  badge,
  stats
}: FeatureCardProps) {
  const router = useRouter();

  const variantClasses = {
    default: 'bg-white dark:bg-botbot-dark border border-gray-100 dark:border-botbot-darker',
    glass: 'bg-white/80 dark:bg-botbot-dark/80 backdrop-blur-md border border-white/20 dark:border-white/10',
    neon: 'bg-gradient-to-br from-gray-900/90 to-black/90 backdrop-blur-md border border-purple-500/30',
    gradient: `bg-gradient-to-br ${gradient} text-white border-0`,
    minimal: 'bg-transparent border-2 border-gray-200 dark:border-gray-700 hover:border-violet-400 dark:hover:border-violet-600'
  };

  return (
    <div
      onClick={() => router.push(href)}
      className={`group relative overflow-hidden rounded-2xl ${variantClasses[variant]} p-5 shadow-lg cursor-pointer transition-all duration-500 hover:scale-[1.03] hover:shadow-2xl`}
      style={{ animationDelay: `${delay}ms` }}
    >
      {/* Animated background effects */}
      {variant === 'neon' && (
        <div className="absolute inset-0 opacity-10">
          <div className={`absolute inset-0 bg-gradient-to-br ${gradient} blur-2xl animate-pulse`} />
        </div>
      )}

      {variant !== 'gradient' && variant !== 'minimal' && (
        <div className={`absolute inset-0 bg-gradient-to-br ${gradient} opacity-0 group-hover:opacity-[0.03] transition-opacity duration-500`} />
      )}

      {/* Badge */}
      {badge && (
        <div className="absolute top-3 right-3">
          <span className={`px-2 py-1 text-[10px] font-bold uppercase tracking-wider rounded-full ${
            variant === 'gradient' ? 'bg-white/20 text-white' : 'bg-violet-100 dark:bg-violet-900/30 text-violet-600 dark:text-violet-400'
          }`}>
            {badge}
          </span>
        </div>
      )}

      <div className="relative z-10">
        <div className="flex items-start justify-between mb-4">
          <div className={`relative ${
            variant === 'gradient' ? 'bg-white/20' :
            variant === 'minimal' ? 'bg-gray-100 dark:bg-gray-800' :
            `bg-gradient-to-br ${gradient}`
          } rounded-xl p-3 shadow-lg group-hover:scale-110 group-hover:rotate-3 transition-all duration-500`}>
            {variant === 'neon' && (
              <div className={`absolute inset-0 bg-gradient-to-br ${gradient} rounded-xl blur animate-pulse`} />
            )}
            <div className="relative">
              {icon}
            </div>
          </div>

          <ArrowUpRight className={`h-4 w-4 ${
            variant === 'gradient' ? 'text-white/70' : 'text-gray-400 dark:text-gray-600'
          } opacity-0 group-hover:opacity-100 transform translate-y-1 group-hover:translate-y-0 transition-all duration-300`} />
        </div>

        <h3 className={`text-base font-bold mb-2 ${
          variant === 'gradient' || variant === 'neon' ? 'text-white' : 'text-gray-900 dark:text-white'
        }`}>
          {title}
        </h3>

        <p className={`text-xs mb-3 ${
          variant === 'gradient' || variant === 'neon' ? 'text-white/80' : 'text-gray-600 dark:text-gray-400'
        } line-clamp-2`}>
          {description}
        </p>

        {/* Stats */}
        {stats && stats.length > 0 && (
          <div className="flex items-center gap-3 mt-auto pt-3 border-t border-gray-100 dark:border-gray-800">
            {stats.map((stat, index) => (
              <div key={index} className="flex-1">
                <p className={`text-[10px] uppercase tracking-wider ${
                  variant === 'gradient' || variant === 'neon' ? 'text-white/60' : 'text-gray-500 dark:text-gray-500'
                }`}>
                  {stat.label}
                </p>
                <p className={`text-sm font-bold ${
                  variant === 'gradient' || variant === 'neon' ? 'text-white' : 'text-gray-900 dark:text-white'
                }`}>
                  {stat.value}
                </p>
              </div>
            ))}
          </div>
        )}
      </div>
    </div>
  );
}

export default function FeatureGrid() {
  const features = [
    {
      icon: <Network className="h-5 w-5 text-white" />,
      title: "Robot Fleet",
      description: "Manage and connect to your entire robot fleet with real-time status monitoring",
      href: "/fleet",
      gradient: "from-violet-500 to-purple-600",
      variant: 'gradient' as const,
    },
    {
      icon: <Home className="h-5 w-5 text-white" />,
      title: "Cockpit",
      description: "Control center for robot operations, telemetry, and real-time diagnostics",
      href: "/cockpit",
      gradient: "from-blue-500 to-cyan-600",
      variant: 'glass' as const,
    },
    {
      icon: <Layout className="h-5 w-5 text-white" />,
      title: "My UI",
      description: "Customize your dashboard with widgets and personalized layouts",
      href: "/my-ui",
      gradient: "from-green-500 to-emerald-600",
      variant: 'glass' as const,
    },
  ];

  return (
    <div className="space-y-4">
      <div className="flex items-center justify-between mb-2">
        <div>
          <h2 className="text-lg font-bold text-gray-900 dark:text-white">Quick Access</h2>
          <p className="text-xs text-gray-600 dark:text-gray-400 mt-1">Your most used features and tools</p>
        </div>
      </div>

      <div className="grid grid-cols-1 md:grid-cols-3 gap-4">
        {features.map((feature, index) => (
          <FeatureCard
            key={feature.href}
            {...feature}
            delay={index * 30}
          />
        ))}
      </div>
    </div>
  );
}