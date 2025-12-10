const path = require('path');
const webpack = require('webpack');

// Conditionally add bundle analyzer plugin
const plugins = [
  // Define environment for better minification
  new webpack.DefinePlugin({
    'process.env.NODE_ENV': JSON.stringify('production')
  })
];

// Add bundle analyzer if ANALYZE environment variable is set
if (process.env.ANALYZE) {
  const BundleAnalyzerPlugin = require('webpack-bundle-analyzer').BundleAnalyzerPlugin;
  plugins.push(new BundleAnalyzerPlugin({
    analyzerMode: 'static',
    openAnalyzer: false,
    reportFilename: 'bundle-report.html'
  }));
}

module.exports = {
  entry: './src/components/ChatWidget.jsx',
  output: {
    path: path.resolve(__dirname, 'dist'),
    filename: 'widget.js',
    library: 'ChatbotWidget',
    libraryTarget: 'umd',
    globalObject: 'this',
    clean: true // Clean dist folder before each build
  },
  module: {
    rules: [
      {
        test: /\.(js|jsx)$/,
        exclude: /node_modules/,
        use: {
          loader: 'babel-loader',
          options: {
            presets: ['@babel/preset-env', '@babel/preset-react'],
            plugins: [
              // Minify and optimize React code
              ['@babel/plugin-transform-react-inline-elements'],
              ['@babel/plugin-transform-react-constant-elements']
            ]
          }
        }
      },
      {
        test: /\.css$/,
        use: [
          // Extract CSS to separate file for better performance
          'style-loader',
          {
            loader: 'css-loader',
            options: {
              modules: false, // Don't use CSS modules for widget
              sourceMap: false // Disable source maps in production
            }
          }
        ]
      }
    ]
  },
  resolve: {
    extensions: ['.js', '.jsx']
  },
  externals: {
    'react': 'React',
    'react-dom': 'ReactDOM'
  },
  optimization: {
    minimize: true,
    sideEffects: false,
    usedExports: true,
    concatenateModules: true,
    splitChunks: {
      chunks: 'all',
      cacheGroups: {
        default: false,
        vendors: false,
        // Since we're building a widget, we'll keep it as a single bundle
        // but optimize it for size
      }
    }
  },
  plugins: plugins,
  // Performance hints
  performance: {
    maxAssetSize: 250000, // 250KB max
    maxEntrypointSize: 250000, // 250KB max
    hints: 'warning'
  },
  stats: {
    colors: true,
    modules: false,
    reasons: false,
    errorDetails: true
  }
};